/* ============================================================================
 * SAMODZIELNA WAGA PASIECZNA: ESP32-S3 + SIM7080G + HX711 + DS18B20
 * Wersja 8.3 (Production-Ready / Fault-Tolerant)
 *
 * GŁÓWNE CECHY I POPRAWKI:
 *  - WDROŻENIE PUNKTU 3 (Twarde czyszczenie przepełnionej kolejki RTC):
 *    Jeśli wygenerowany JSON przekroczy dozwolony limit znaków z powodu
 *    nagromadzenia starych błędów (np. wielogodzinna awaria serwera),
 *    kod nie tylko skraca przesyłany pakiet, ale BEZPOWROTNIE USUWA
 *    najstarsze błędy z trwałej pamięci RTC. Eliminuje to zjawisko
 *    "zablokowania wagi" (nieskończona pętla ERR_JSON_SERIALIZE_FAIL).
 *  - DEFINITYWNA NAPRAWA CSQ DLA LTE CAT-M (Kaskada AT+CPSI? z ułamkami dBm).
 *  - OBSŁUGA CZUJNIKA TEMPERATURY DS18B20 (1-Wire na GPIO 4).
 *  - TENSOMETR HX711 ZE ŚREDNIĄ UCIĘTĄ (16 próbek, odrzucenie skrajności).
 * ==========================================================================*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>
#include "esp_system.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"

// --- BIBLIOTEKI OBSŁUGI DS18B20 ---
#include <OneWire.h>
#include <DallasTemperature.h>

#if __has_include("esp_arduino_version.h")
  #include "esp_arduino_version.h"
#endif

// ============================================================================
// TRYB PRACY: DEBUG vs PRODUKCJA
// 1 = TRYB DEVELOPERSKI (pełne logi na monitor szeregowy USB)
// 0 = TRYB PRODUKCYJNY (Serial całkowicie wycięty przez kompilator, oszczędność prądu)
// ============================================================================
#define DEBUG_ENABLED 1

#if !DEBUG_ENABLED
class NullStream {
public:
  void begin(unsigned long, uint32_t = 0, int8_t = -1, int8_t = -1, bool = true, unsigned long = 0) {}
  void end() {}
  void flush() {}
  void setTxTimeoutMs(uint32_t = 0) {}
  operator bool() const { return false; }

  template<typename... Args> void print(Args&&...) {}
  template<typename... Args> void println(Args&&...) {}
  void print() {}
  void println() {}
};

static NullStream _nullSerial;
#ifdef Serial
  #undef Serial
#endif
#define Serial _nullSerial
#endif

// ============================================================================
// KONFIGURACJA WAGI, APN & ENDPOINT API
// ============================================================================
#define FIRMWARE_VERSION  "8.3"
#define SCALE_ID          "3fa85f64-5717-4562-b3fc-2c963f66afa6"
#define MODEM_APN         "iot"
#define API_HOST          "http://pasieka.loca.lt"
#define API_PATH          "/api/reports"

#define JSON_BUF_SIZE     1792
#define HTTP_BODYLEN      2048
static_assert(JSON_BUF_SIZE <= HTTP_BODYLEN, "JSON_BUF_SIZE nie może przekraczać HTTP_BODYLEN");

#define TIME_TO_SLEEP          120     // [s] cykl uśpienia (testowo 120 s, docelowo 600 s)
#define TIME_TO_SLEEP_LOWBATT  1800    // [s] cykl w trybie ochrony baterii
#define TIME_RESYNC_S          (6UL * 3600UL)  // [s] co ile odświeżać zegar ESP32
#define uS_TO_S_FACTOR         1000000ULL

Preferences prefs;
unsigned int msgCounter = 0;

RTC_DATA_ATTR uint16_t lastHttpTimeMs = 0;
RTC_DATA_ATTR time_t   lastTimeSyncEpoch = 0;

// --- Konfiguracja pinów HX711 ---
#define HX711_DT  10
#define HX711_SCK 1

// --- Parametry tensometru ---
const float WSPOLCZYNNIK_KALIBRACJI = 2000.0f;
const long  OFFSET_ZERA             = 257388L;
#define LICZBA_ODCZYTOW   16   // 16 próbek dla średniej uciętej
#define HX711_SETTLE_MS   150

// --- Konfiguracja czujnika temperatury DS18B20 ---
#define ONE_WIRE_PIN 4         // GPIO magistrali 1-Wire z rezystorem 2k do 3.3V
const float DS18B20_OFFSET = 0.0f; // Wpisz np. -3.0f, jeśli czujnik stale zawyża o 3°C

OneWire oneWireBus(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWireBus);

// --- Konfiguracja SIM7080G ---
#define SIM_RX_PIN        17
#define SIM_TX_PIN        18
#define PWRKEY_PIN        14
#define PWRKEY_ACTIVE_LOW true
#define PWRKEY_ON_MS      1200
#define DEBUG_NO_SLEEP    true
#define NET_MODE_CNMP     38        // 38 = tylko LTE
#define NET_MODE_CMNB     1         // 1 = tylko Cat-M (bez NB-IoT)
#define CATM_BANDS        "3,8,20"  // pasma Cat-M dla AT+CBANDCFG

// ============================================================================
// TIMEOUTY SIECIOWE (60 SEKUND)
// ============================================================================
#define NET_REG_TIMEOUT_MS 60000UL  // 60 s na rejestrację LTE

// --- Ochrona baterii (histereza) ---
#define VBAT_CUTOFF_V     3.40f     // poniżej: wejście w tryb ochrony (bez transmisji)
#define VBAT_RECOVER_V    3.60f     // powyżej: wyjście z trybu ochrony
#define VBAT_SOURCE_MAX17048  0     // 0 = AT+CBC; 1 = układ zewnętrzny I2C
#define MAX17048_SDA          8     
#define MAX17048_SCL          9     
#define MAX17048_ADDR         0x36

bool networkReady = false;

bool ensurePdpActive(uint8_t* outErr);

// ============================================================================
// SYSTEM KODÓW BŁĘDÓW
// ============================================================================
enum StatusCode : uint8_t {
  ERR_OK                    = 0,
  ERR_HX711_TIMEOUT         = 11,
  ERR_HX711_NAN             = 12,
  ERR_MODEM_NO_AT           = 21,
  ERR_BATTERY_CRITICAL      = 22,
  ERR_BATTERY_BROWNOUT      = 23,
  ERR_WATCHDOG_RESET        = 24,
  ERR_HARDWARE_CRASH        = 25,
  ERR_NET_REG_TIMEOUT       = 31,
  ERR_PDP_ACT_FAILED        = 41,
  ERR_IP_LEASE_TIMEOUT      = 42,
  ERR_TIME_SYNC_FAILED      = 43,
  ERR_TCP_CONN_FAILED       = 51,
  ERR_HTTP_PROMPT_TIMEOUT   = 52,
  ERR_HTTP_BODY_REJECTED    = 53,
  ERR_HTTP_REQ_TIMEOUT      = 54,
  ERR_HTTP_STATUS_NOT_2XX   = 55,
  ERR_JSON_SERIALIZE_FAIL   = 61
};

const char* getStatusCodeName(uint8_t code) {
  switch (code) {
    case ERR_OK:                  return "ERR_OK";
    case ERR_HX711_TIMEOUT:       return "ERR_HX711_TIMEOUT";
    case ERR_HX711_NAN:           return "ERR_HX711_NAN";
    case ERR_MODEM_NO_AT:         return "ERR_MODEM_NO_AT";
    case ERR_BATTERY_CRITICAL:    return "ERR_BATTERY_CRITICAL";
    case ERR_BATTERY_BROWNOUT:    return "ERR_BATTERY_BROWNOUT";
    case ERR_WATCHDOG_RESET:      return "ERR_WATCHDOG_RESET";
    case ERR_HARDWARE_CRASH:      return "ERR_HARDWARE_CRASH";
    case ERR_NET_REG_TIMEOUT:     return "ERR_NET_REG_TIMEOUT";
    case ERR_PDP_ACT_FAILED:      return "ERR_PDP_ACT_FAILED";
    case ERR_IP_LEASE_TIMEOUT:    return "ERR_IP_LEASE_TIMEOUT";
    case ERR_TIME_SYNC_FAILED:    return "ERR_TIME_SYNC_FAILED";
    case ERR_TCP_CONN_FAILED:     return "ERR_TCP_CONN_FAILED";
    case ERR_HTTP_PROMPT_TIMEOUT: return "ERR_HTTP_PROMPT_TIMEOUT";
    case ERR_HTTP_BODY_REJECTED:  return "ERR_HTTP_BODY_REJECTED";
    case ERR_HTTP_REQ_TIMEOUT:    return "ERR_HTTP_REQ_TIMEOUT";
    case ERR_HTTP_STATUS_NOT_2XX: return "ERR_HTTP_STATUS_NOT_2XX";
    case ERR_JSON_SERIALIZE_FAIL: return "ERR_JSON_SERIALIZE_FAIL";
    default:                      return "ERR_UNKNOWN";
  }
}

// ============================================================================
// PUNKTY KONTROLNE (BREADCRUMBS)
// ============================================================================
enum CycleStep : uint8_t {
  ST_UNKNOWN = 0,
  ST_BOOT,
  ST_HX711,
  ST_MODEM_ON,
  ST_MODEM_CFG,
  ST_BATTERY,
  ST_RADIO_CFG,
  ST_NET_REG,
  ST_TIME_SYNC,
  ST_JSON,
  ST_PDP,
  ST_HTTP_CONF,
  ST_SHCONN,
  ST_SHBOD,
  ST_SHREQ,
  ST_SHREAD,
  ST_MODEM_OFF,
  ST_SLEEP
};

const char* getStepName(uint8_t s) {
  switch (s) {
    case ST_BOOT:      return "BOOT";
    case ST_HX711:     return "HX711";
    case ST_MODEM_ON:  return "MODEM_ON";
    case ST_MODEM_CFG: return "MODEM_CFG";
    case ST_BATTERY:   return "BATTERY";
    case ST_RADIO_CFG: return "RADIO_CFG";
    case ST_NET_REG:   return "NET_REG";
    case ST_TIME_SYNC: return "TIME_SYNC";
    case ST_JSON:      return "JSON";
    case ST_PDP:       return "PDP";
    case ST_HTTP_CONF: return "HTTP_CONF";
    case ST_SHCONN:    return "SHCONN";
    case ST_SHBOD:     return "SHBOD";
    case ST_SHREQ:     return "SHREQ";
    case ST_SHREAD:    return "SHREAD";
    case ST_MODEM_OFF: return "MODEM_OFF";
    case ST_SLEEP:     return "SLEEP";
    default:           return "UNKNOWN";
  }
}

// ============================================================================
// STAN TRWAŁY W RTC RAM (CZARNA SKRZYNKA)
// ============================================================================
struct ErrorLogEntry {
  uint32_t report_id;
  char timestamp[35];
  uint8_t code;
  int16_t detail;
  char msg[36];
  float vbat_v;
  int8_t csq;
};

#define MAX_ERR_LOGS 8

struct PersistState {
  uint32_t magic;
  uint8_t  step;
  uint8_t  errCount;
  uint8_t  lowBatt;
  uint8_t  reserved;
  ErrorLogEntry errQueue[MAX_ERR_LOGS];
};

#define PS_MAGIC_BASE 0x57A6E062UL
#define PS_MAGIC      (PS_MAGIC_BASE ^ (uint32_t)sizeof(PersistState))

RTC_NOINIT_ATTR PersistState ps;
static uint8_t stepAtReset = ST_UNKNOWN;
static esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;

const char* getResetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "INNY";
  }
}

static inline void markStep(uint8_t s) { ps.step = s; }

void initPersistState() {
  if (ps.magic != PS_MAGIC) {
    memset(&ps, 0, sizeof(ps));
    ps.magic = PS_MAGIC;
    stepAtReset = ST_UNKNOWN;
    return;
  }
  stepAtReset = ps.step;
  if (ps.errCount > MAX_ERR_LOGS) ps.errCount = 0;
  if (ps.lowBatt > 1) ps.lowBatt = 0;
  for (uint8_t i = 0; i < MAX_ERR_LOGS; i++) {
    ps.errQueue[i].timestamp[sizeof(ps.errQueue[i].timestamp) - 1] = '\0';
    ps.errQueue[i].msg[sizeof(ps.errQueue[i].msg) - 1] = '\0';
  }
}

float sanitizeFloat(float val, float fallback = 0.0f) {
  if (isnan(val) || isinf(val)) return fallback;
  return val;
}

void sanitizeErrorMessage(char* dest, const char* src, size_t maxLen) {
  if (!dest || maxLen == 0) return;
  if (!src || strlen(src) == 0) {
    dest[0] = '\0';
    return;
  }
  size_t j = 0;
  for (size_t i = 0; src[i] != '\0' && j < maxLen - 1; i++) {
    char c = src[i];
    if (c == '"' || c == '\\' || c == '\'' || c == '\r' || c == '\n' || c == '\t') c = ' ';
    if (isprint((unsigned char)c)) {
      if (c == ' ' && j > 0 && dest[j - 1] == ' ') continue;
      dest[j++] = c;
    }
  }
  while (j > 0 && dest[j - 1] == ' ') j--;
  dest[j] = '\0';
}

void recordError(uint32_t repId, const char* ts, uint8_t code, int16_t detail, const char* errorMsg, float vbat, int csq) {
  if (code == ERR_OK) return;
  if (ps.errCount > MAX_ERR_LOGS) ps.errCount = 0;

  char cleanMsg[sizeof(ps.errQueue[0].msg)] = {0};
  sanitizeErrorMessage(cleanMsg, errorMsg, sizeof(cleanMsg));
  const char* validTs = (ts && ts[0]) ? ts : "null";

  ErrorLogEntry* e;
  if (ps.errCount > 0 && ps.errQueue[ps.errCount - 1].report_id == repId) {
    e = &ps.errQueue[ps.errCount - 1];
  } else {
    if (ps.errCount >= MAX_ERR_LOGS) {
      memmove(&ps.errQueue[0], &ps.errQueue[1], sizeof(ErrorLogEntry) * (MAX_ERR_LOGS - 1));
      ps.errCount = MAX_ERR_LOGS - 1;
    }
    e = &ps.errQueue[ps.errCount++];
    e->report_id = repId;
    strncpy(e->timestamp, validTs, sizeof(e->timestamp) - 1);
    e->timestamp[sizeof(e->timestamp) - 1] = '\0';
  }

  e->code = code;
  e->detail = detail;
  e->vbat_v = sanitizeFloat(vbat, 0.0f);
  e->csq = (int8_t)csq;
  strncpy(e->msg, cleanMsg, sizeof(e->msg) - 1);
  e->msg[sizeof(e->msg) - 1] = '\0';
}

void checkHardwareResetReason(uint32_t lastReportId) {
  esp_reset_reason_t reason = bootResetReason;
  uint8_t code;
  const char* kind;

  switch (reason) {
    case ESP_RST_BROWNOUT: code = ERR_BATTERY_BROWNOUT; kind = "BROWNOUT"; break;
    case ESP_RST_TASK_WDT:
    case ESP_RST_INT_WDT:
    case ESP_RST_WDT:      code = ERR_WATCHDOG_RESET; kind = "WDT"; break;
    case ESP_RST_PANIC:    code = ERR_HARDWARE_CRASH; kind = "PANIC"; break;
    default: return;
  }

  char msg[36];
  snprintf(msg, sizeof(msg), "%s(%d)@%s", kind, (int)reason, getStepName(stepAtReset));
  Serial.print(F("⚠️ [DIAG] Twardy restart: "));
  Serial.println(msg);
  recordError(lastReportId, "null", code, (int16_t)stepAtReset, msg, 0.0f, 99);
}

// ============================================================================
// WATCHDOG SPRZĘTOWY (TG0WDT) - 90 SEKUND
// ============================================================================

void setupWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {
    .timeout_ms = 90000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&cfg);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(90, true);
  esp_task_wdt_add(NULL);
#endif
  Serial.println(F("🐕 Watchdog skonfigurowany (limit 90 s)."));
}

void delaySafe(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    delay(10);
    yield();
    esp_task_wdt_reset();
  }
}

// ============================================================================
// ODPORNY POMIAR TEMPERATURY DS18B20 Z FILTRACJĄ I KALIBRACJĄ
// ============================================================================

float zmierzTemperatureDS18B20() {
  Serial.println(F("🌡️ [DS18B20] Weryfikacja magistrali 1-Wire i odczyt temperatury..."));
  ds18b20.begin();

  int devCount = ds18b20.getDeviceCount();
  if (devCount == 0) {
    Serial.println(F("❌ [DS18B20] Brak czujnika na magistrali! Sprawdź kabel i rezystor 2k."));
    return NAN;
  }

  ds18b20.setResolution(12);
  ds18b20.setWaitForConversion(false);

  float tempC = NAN;

  // Do 2 prób odczytu (odrzucenie 85°C przy resecie rejestru i -127°C)
  for (int tryIdx = 1; tryIdx <= 2; tryIdx++) {
    ds18b20.requestTemperatures();
    delaySafe(750); // Czas na konwersję 12-bit

    float rawTemp = ds18b20.getTempCByIndex(0);

    if (rawTemp != DEVICE_DISCONNECTED_C && rawTemp != 85.0f && rawTemp >= -55.0f && rawTemp <= 125.0f) {
      tempC = rawTemp + DS18B20_OFFSET;
      break;
    }

    Serial.print(F("⚠️ [DS18B20] Próba #"));
    Serial.print(tryIdx);
    Serial.print(F(" nieudana (zwrócono: "));
    Serial.print(rawTemp);
    Serial.println(F(" °C) — ponawiam pomiar..."));
    delaySafe(200);
  }

  if (isnan(tempC)) {
    Serial.println(F("❌ [DS18B20] Nie udało się uzyskać prawidłowej temperatury."));
    return NAN;
  }

  Serial.print(F("✅ [DS18B20] Odczyt temperatury: "));
  Serial.print(tempC, 2);
  Serial.print(F(" °C"));
  if (DS18B20_OFFSET != 0.0f) {
    Serial.print(F(" (z uwzględnieniem offsetu "));
    Serial.print(DS18B20_OFFSET, 2);
    Serial.print(F(" °C)"));
  }
  Serial.println();

  return tempC;
}

// ============================================================================
// OBSŁUGA PRZETWORNIKA HX711 (ŚREDNIA UCIĘTA Z 16 PRÓBEK)
// ============================================================================

void setupHX711Pins() {
  pinMode(HX711_DT, INPUT);
  pinMode(HX711_SCK, OUTPUT);
  digitalWrite(HX711_SCK, LOW);
}

void powerDownHX711() {
  digitalWrite(HX711_SCK, HIGH);
  delayMicroseconds(70);
}

void powerUpHX711() {
  digitalWrite(HX711_SCK, LOW);
}

bool readHX711RawSafe(long* outVal, uint32_t timeoutMs = 200) {
  uint32_t start = millis();
  while (digitalRead(HX711_DT) == HIGH) {
    if (millis() - start > timeoutMs) return false;
    delay(2);
    yield();
    esp_task_wdt_reset();
  }

  uint32_t data = 0;
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    data = (data << 1) | (uint32_t)digitalRead(HX711_DT);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(HX711_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX711_SCK, LOW);
  delayMicroseconds(1);

  if (data & 0x800000UL) data |= 0xFF000000UL;
  if (outVal) *outVal = (long)(int32_t)data;
  return true;
}

float zmierzWage(long* outRaw, long* outNoise, uint8_t* outErr) {
  gpio_hold_dis((gpio_num_t)HX711_SCK);

  Serial.println(F("⚖️ [Waga] Inicjalizacja magistrali i odczyt próbek HX711 (16 próbek)..."));
  setupHX711Pins();
  powerUpHX711();
  delaySafe(HX711_SETTLE_MS);

  long samples[LICZBA_ODCZYTOW];
  int validCount = 0;

  for (int i = 0; i < LICZBA_ODCZYTOW; i++) {
    long singleSample = 0;
    if (readHX711RawSafe(&singleSample, 200)) {
      samples[validCount++] = singleSample;
    } else {
      Serial.print(F("⚠️ [HX711] Timeout próbki #"));
      Serial.println(i + 1);
    }
    yield();
    esp_task_wdt_reset();
  }

  powerDownHX711();

  if (validCount < 10) {
    Serial.println(F("❌ BŁĄD: Zbyt mało poprawnych próbek z HX711."));
    if (outErr) *outErr = ERR_HX711_TIMEOUT;
    if (outRaw) *outRaw = 0;
    if (outNoise) *outNoise = 0;
    return NAN;
  }

  // Sortowanie próbek dla średniej uciętej
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = 0; j < validCount - i - 1; j++) {
      if (samples[j] > samples[j + 1]) {
        long temp = samples[j];
        samples[j] = samples[j + 1];
        samples[j + 1] = temp;
      }
    }
  }

  long noise = samples[validCount - 1] - samples[0];

  // Odrzucenie 3 najniższych i 3 najwyższych wartości (tłumienie wiatru i drgań)
  int trimCount = 3;
  long rawSum = 0;
  int usedCount = 0;

  for (int i = trimCount; i < (validCount - trimCount); i++) {
    rawSum += samples[i];
    usedCount++;
  }

  long rawAvg = rawSum / usedCount;

  if (outRaw) *outRaw = rawAvg;
  if (outNoise) *outNoise = noise;

  float weight = (float)(rawAvg - OFFSET_ZERA) / WSPOLCZYNNIK_KALIBRACJI;
  weight = sanitizeFloat(weight, -99.99f);

  Serial.print(F("✅ Zmierzona waga: "));
  Serial.print(weight, 2);
  Serial.print(F(" kg (RAW: "));
  Serial.print(rawAvg);
  Serial.print(F(", szum: "));
  Serial.print(noise);
  Serial.print(F(", uśredniono próbek: "));
  Serial.print(usedCount);
  Serial.print(F("/"));
  Serial.print(validCount);
  Serial.println(F(")"));

  return weight;
}

// ============================================================================
// KOMUNIKACJA UART Z MODEMEM (Serial2)
// ============================================================================

#define AT_BUF_SIZE 512
static char atBuf[AT_BUF_SIZE];

const char* sendAT(const char* cmd, uint32_t timeout = 2000, bool debug = true) {
  if (debug) {
    Serial.print(F(" > "));
    Serial.println(cmd);
  }

  int flushLimit = 1000;
  while (Serial2.available() && flushLimit > 0) {
    Serial2.read();
    flushLimit--;
  }

  Serial2.println(cmd);

  size_t n = 0;
  atBuf[0] = '\0';
  uint32_t start = millis();
  int consecutiveReads = 0;

  while (millis() - start < timeout) {
    consecutiveReads = 0;
    while (Serial2.available() && consecutiveReads < 100) {
      char c = (char)Serial2.read();
      consecutiveReads++;
      if (n < AT_BUF_SIZE - 1) {
        atBuf[n++] = c;
        atBuf[n] = '\0';
      }
    }

    if (strstr(atBuf, "OK\r\n") || strstr(atBuf, "ERROR\r\n")) break;
    if (strstr(atBuf, "+CME ERROR")) break;

    delay(5);
    yield();
    esp_task_wdt_reset();
  }

  if (debug) {
    Serial.print(F(" < "));
    Serial.println(atBuf[0] ? atBuf : "(brak odpowiedzi / TIMEOUT)");
    Serial.println(F("----------------------------------------"));
  }
  return atBuf;
}

const char* readUntil(const char* token, uint32_t timeoutMs) {
  size_t n = strlen(atBuf);
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (n < AT_BUF_SIZE - 1) {
        atBuf[n++] = c;
        atBuf[n] = '\0';
      }
    }
    const char* p = strstr(atBuf, token);
    if (p && strstr(p, "\r\n")) return p;
    delay(10);
    yield();
    esp_task_wdt_reset();
  }
  return NULL;
}

bool respContains(const char* cmd, const char* expected, uint32_t timeout = 1000) {
  return strstr(sendAT(cmd, timeout, false), expected) != NULL;
}

bool isModemAlive() {
  for (int i = 0; i < 3; i++) {
    if (strstr(sendAT("AT", 800, false), "OK")) return true;
    delaySafe(150);
  }
  return false;
}

// ============================================================================
// STEROWANIE ZASILANIEM MODEMU
// ============================================================================

static inline void pwrkeyRelease() {
  pinMode(PWRKEY_PIN, INPUT);
}

static inline void pwrkeyPress() {
  digitalWrite(PWRKEY_PIN, PWRKEY_ACTIVE_LOW ? LOW : HIGH);
  pinMode(PWRKEY_PIN, OUTPUT);
}

void pulsePwrKey(uint16_t holdMs) {
  pwrkeyRelease();
  delaySafe(50);
  pwrkeyPress();
  delaySafe(holdMs);
  pwrkeyRelease();
}

bool configureModem() {
  sendAT("ATE0", 800, false);
  sendAT("AT+CMEE=2", 800, false);
  sendAT("AT+CLTS=1", 800, false);

#if DEBUG_NO_SLEEP
  sendAT("AT+CSCLK=0", 800, false);
#else
  sendAT("AT+CSCLK=2", 800, false);
#endif
  return true;
}

void ensureRadioConfig() {
  char expected[40];

  snprintf(expected, sizeof(expected), "+CNMP: %d", NET_MODE_CNMP);
  bool okCnmp = respContains("AT+CNMP?", expected);

  snprintf(expected, sizeof(expected), "+CMNB: %d", NET_MODE_CMNB);
  bool okCmnb = respContains("AT+CMNB?", expected);

  bool okBand = respContains("AT+CBANDCFG?", "\"CAT-M\"," CATM_BANDS "\r");

  if (okCnmp && okCmnb && okBand) {
    Serial.println(F("📡 Konfiguracja radia OK (LTE / Cat-M / pasma " CATM_BANDS ")."));
    return;
  }

  Serial.print(F("📡 Korekta konfiguracji radia: CNMP="));
  Serial.print(okCnmp ? "ok" : "ZMIANA");
  Serial.print(F(", CMNB="));
  Serial.print(okCmnb ? "ok" : "ZMIANA");
  Serial.print(F(", CBANDCFG="));
  Serial.println(okBand ? "ok" : "ZMIANA");

  char cmd[48];
  sendAT("AT+CFUN=0", 10000, true);
  snprintf(cmd, sizeof(cmd), "AT+CNMP=%d", NET_MODE_CNMP);
  sendAT(cmd, 1500, true);
  snprintf(cmd, sizeof(cmd), "AT+CMNB=%d", NET_MODE_CMNB);
  sendAT(cmd, 1500, true);
  sendAT("AT+CBANDCFG=\"CAT-M\"," CATM_BANDS, 1500, true);
  sendAT("AT+CFUN=1", 10000, true);
}

// Rozruch v7.5
bool ensureModemOn() {
  pwrkeyRelease();

  Serial2.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delaySafe(100);

  Serial.print(F("🔍 Weryfikacja stanu modemu... "));
  if (isModemAlive()) {
    Serial.println(F("✅ Modem już włączony i aktywny — PWRKEY nietknięty!"));
    return true;
  }

  Serial.println(F("ℹ️ Modem milczy."));
  Serial.println(F("⚡ Uruchamianie modemu (impuls PWRKEY 1.2s)..."));
  pulsePwrKey(PWRKEY_ON_MS);

  Serial.print(F("⏳ Czas na rozruch sprzętowy SIMCom: "));
  for (int s = 1; s <= 4; s++) {
    delaySafe(1000);
    Serial.print(F("["));
    Serial.print(s);
    Serial.print(F("s]"));
  }
  Serial.println();

  Serial.print(F("⏳ Oczekiwanie na synchronizację AT: "));
  uint32_t atStart = millis();
  uint32_t lastPrint = millis();
  bool atReady = false;

  while (millis() - atStart < 10000UL) {
    if (strstr(sendAT("AT", 200, false), "OK")) {
      atReady = true;
      break;
    }
    if (millis() - lastPrint >= 1000) {
      uint32_t el = (millis() - atStart) / 1000;
      Serial.print(F("["));
      Serial.print(el);
      Serial.print(F("s]"));
      lastPrint = millis();
    } else {
      Serial.print('.');
    }
    delaySafe(200);
  }

  if (atReady) {
    Serial.println(F("\n✅ Modem zsynchronizowany i w pełni gotowy!"));
    return true;
  }

  Serial.println(F("\n❌ Timeout odpowiedzi AT po włączeniu zasilania."));
  return false;
}

// Wyłączanie modemu
void turnOffModem() {
  markStep(ST_MODEM_OFF);
  Serial.println(F("🛑 Bezpieczne wyłączanie modemu LTE..."));
  
  sendAT("AT+SHDISC", 800, false);
  while (Serial2.available()) Serial2.read();

  Serial.println(F(" > AT+CPOWD=1"));
  Serial2.println(F("AT+CPOWD=1"));
  Serial2.flush();

  uint32_t tStart = millis();
  bool powerDownConfirmed = false;
  char pwrBuf[64] = {0};
  size_t pIdx = 0;

  while (millis() - tStart < 4500UL) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (pIdx < sizeof(pwrBuf) - 1) {
        pwrBuf[pIdx++] = c;
        pwrBuf[pIdx] = '\0';
      }
    }
    if (strstr(pwrBuf, "NORMAL POWER DOWN") || strstr(pwrBuf, "POWER DOWN")) {
      powerDownConfirmed = true;
      break;
    }
    delay(10);
    yield();
    esp_task_wdt_reset();
  }

  if (powerDownConfirmed) {
    Serial.println(F("✅ Modem potwierdził: NORMAL POWER DOWN."));
    delaySafe(1200);
  } else {
    Serial.print(F("⚠️ Brak potwierdzenia (odebrano: '"));
    Serial.print(pwrBuf);
    Serial.println(F("'). Wymuszam wyłączenie sprzętowe (PWRKEY)..."));
    pulsePwrKey(1500);
    delaySafe(2000);
  }

  pinMode(SIM_RX_PIN, INPUT_PULLUP);
  pinMode(SIM_TX_PIN, INPUT_PULLUP);
  Serial2.end();

  Serial.println(F("🔒 Zasilanie modemu odcięte, dioda NET wygaszona."));
}

// ============================================================================
// BATERIA I STAN SIECI
// ============================================================================

float getBatteryVoltage(bool print = false) {
  const char* r = sendAT("AT+CBC", 800, false);
  const char* p = strstr(r, "+CBC:");
  if (!p) return 0.0f;
  const char* c1 = strchr(p, ',');
  if (!c1) return 0.0f;
  const char* c2 = strchr(c1 + 1, ',');
  if (!c2) return 0.0f;
  float voltageV = sanitizeFloat(atof(c2 + 1) / 1000.0f, 0.0f);
  if (print) {
    Serial.print(F("🔋 BATERIA (modem): "));
    Serial.print(atoi(c1 + 1));
    Serial.print(F("% (VBAT: "));
    Serial.print(voltageV, 2);
    Serial.println(F(" V)"));
  }
  return voltageV;
}

#if VBAT_SOURCE_MAX17048
float readMax17048Voltage() {
  Wire.begin(MAX17048_SDA, MAX17048_SCL);
  Wire.beginTransmission(MAX17048_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) { Wire.end(); return 0.0f; }
  if (Wire.requestFrom((uint8_t)MAX17048_ADDR, (uint8_t)2) != 2) { Wire.end(); return 0.0f; }
  uint8_t msb = (uint8_t)Wire.read();
  uint8_t lsb = (uint8_t)Wire.read();
  Wire.end();
  float v = (float)(((uint16_t)msb << 8) | lsb) * 78.125e-6f;
  Serial.print(F("🔋 BATERIA (MAX17048): "));
  Serial.print(v, 3);
  Serial.println(F(" V"));
  return v;
}
#endif

bool batteryBlocksTx(float vbat, uint32_t repId, const char* ts) {
  if (vbat < 0.5f) {
    Serial.println(F("⚠️ [BAT] Brak odczytu napięcia — nie blokuję transmisji."));
    return false;
  }

  if (ps.lowBatt) {
    if (vbat >= VBAT_RECOVER_V) {
      ps.lowBatt = 0;
      Serial.println(F("🔋 [BAT] Napięcie wróciło powyżej progu RECOVER — wznawiam transmisję."));
      return false;
    }
    Serial.println(F("🪫 [BAT] Nadal tryb ochrony baterii — pomijam transmisję."));
    return true;
  }

  if (vbat < VBAT_CUTOFF_V) {
    ps.lowBatt = 1;
    recordError(repId, ts, ERR_BATTERY_CRITICAL, (int16_t)(vbat * 100.0f), "LOW_VOLT", vbat, 99);
    Serial.println(F("🪫 [BAT] Napięcie poniżej CUTOFF — wchodzę w tryb ochrony baterii."));
    return true;
  }
  return false;
}

// ============================================================================
// WERSJA 8.3: PARSER AT+CPSI? Z OBSŁUGĄ JEDNOSTEK DZIESIĘTNYCH (0.1 dBm)
// ============================================================================
int getSignalQuality() {
  // 1. Sprawdź, czy modem zwraca standardowe CSQ
  const char* r = sendAT("AT+CSQ", 800, false);
  const char* p = strstr(r, "+CSQ:");
  if (p) {
    int val = atoi(p + 5);
    if (val != 99 && val > 0) return val;
  }

  // 2. Jeśli CSQ zwraca 99,99 (standard Cat-M), odpytujemy profil komórki
  r = sendAT("AT+CPSI?", 1000, false);
  p = strstr(r, "+CPSI:");
  if (p && !strstr(p, "NO SERVICE")) {
    const char* s = p;
    int commaCount = 0;
    const char* pRsrp = NULL;
    const char* pRssi = NULL;

    while (*s && *s != '\r' && *s != '\n') {
      if (*s == ',') {
        commaCount++;
        if (commaCount == 11) pRsrp = s + 1; // 11. parametr: RSRP
        if (commaCount == 12) pRssi = s + 1; // 12. parametr: RSSI
      }
      s++;
    }

    // Odczyt RSSI w dziesiątych dBm (np. -650 = -65.0 dBm)
    if (pRssi) {
      int rssiRaw = atoi(pRssi);
      if (rssiRaw != 0) {
        float rssiDbm = (rssiRaw < -150) ? ((float)rssiRaw / 10.0f) : (float)rssiRaw;
        int csq = (int)((rssiDbm + 113.0f) / 2.0f + 0.5f);
        if (csq < 1) csq = 1;
        if (csq > 31) csq = 31;
        return csq;
      }
    }

    // Przeliczenie RSRP
    if (pRsrp) {
      int rsrpRaw = atoi(pRsrp);
      if (rsrpRaw != 0) {
        float rsrpDbm = (rsrpRaw < -150) ? ((float)rsrpRaw / 10.0f) : (float)rsrpRaw;
        int csq = (int)((rsrpDbm + 113.0f) / 2.0f + 0.5f);
        if (csq < 1) csq = 1;
        if (csq > 31) csq = 31;
        return csq;
      }
    }
  }

  return 99; // Sygnał rzeczywiście niedostępny
}

int checkSignalQuality() {
  int rssi = getSignalQuality();
  Serial.print(F("📶 Zasięg sieci (CSQ): "));
  Serial.print(rssi);
  if (rssi == 99 || rssi == 0) {
    Serial.println(F(" (brak sygnału / nieustalony - 99)"));
  } else {
    Serial.print(F(" (ok. "));
    Serial.print(-113 + rssi * 2);
    Serial.println(F(" dBm)"));
  }
  return rssi;
}

int ceregStatus() {
  const char* r = sendAT("AT+CEREG?", 1200, false);
  const char* p = strstr(r, "+CEREG:");
  if (!p) return -1;
  const char* comma = strchr(p, ',');
  if (!comma) return -1;
  return atoi(comma + 1);
}

bool isRegistered() {
  int s = ceregStatus();
  return (s == 1 || s == 5);
}

const char* ceregStatusDesc(int s) {
  switch (s) {
    case 0: return "brak rejestracji (modem nie szuka)";
    case 1: return "zarejestrowano (sieć domowa)";
    case 2: return "szukanie stacji bazowej...";
    case 3: return "odrzucono przez operatora!";
    case 4: return "stan nieznany";
    case 5: return "zarejestrowano (roaming)";
    default: return "brak odpowiedzi AT modemu";
  }
}

// ============================================================================
// ZEGAR CZASU RZECZYWISTEGO (ESP32 RTC)
// ============================================================================

static const int daysBeforeMonth[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

time_t calculateUtcEpoch(int year, int month, int day, int hour, int min, int sec, char tzSign, int tzQuarters) {
  int y = year - 1970;
  int leapDays = (year - 1969) / 4;
  int days = y * 365 + leapDays + daysBeforeMonth[month - 1] + (day - 1);
  if (month > 2 && (year % 4 == 0)) days++;
  time_t epoch = (time_t)days * 86400 + (time_t)hour * 3600 + (time_t)min * 60 + sec;
  int tzOffsetSec = tzQuarters * 15 * 60;
  if (tzSign == '+') epoch -= tzOffsetSec;
  else if (tzSign == '-') epoch += tzOffsetSec;
  return epoch;
}

bool isEspTimeValid() {
  time_t now = time(NULL);
  return (now > 1704067200);
}

bool needTimeSync() {
  if (!isEspTimeValid()) return true;
  time_t now = time(NULL);
  if (lastTimeSyncEpoch == 0 || now < lastTimeSyncEpoch) return true;
  return (now - lastTimeSyncEpoch) >= (time_t)TIME_RESYNC_S;
}

bool syncEspRtcFromModem() {
  const char* r = sendAT("AT+CCLK?", 800, false);
  const char* p = strstr(r, "+CCLK: \"");
  if (!p) return false;
  p += 8;

  int yy, MM, dd, hh, mm, ss, tzQuarters;
  char tzSign;

  if (sscanf(p, "%2d/%2d/%2d,%2d:%2d:%2d%c%2d",
             &yy, &MM, &dd, &hh, &mm, &ss, &tzSign, &tzQuarters) != 8) {
    return false;
  }

  if (yy < 24 || yy > 40) return false;
  if (MM < 1 || MM > 12 || dd < 1 || dd > 31) return false;
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 60) return false;
  if ((tzSign != '+' && tzSign != '-') || tzQuarters < 0 || tzQuarters > 56) return false;

  time_t utcEpoch = calculateUtcEpoch(2000 + yy, MM, dd, hh, mm, ss, tzSign, tzQuarters);
  bool hadValidTime = isEspTimeValid();
  time_t before = time(NULL);

  struct timeval tv = { .tv_sec = utcEpoch, .tv_usec = 0 };
  settimeofday(&tv, NULL);

  Serial.print(F("🕒 Zegar ESP32 zsynchronizowany z modemem"));
  if (hadValidTime) {
    Serial.print(F(" (korekta dryfu: "));
    Serial.print((long)(utcEpoch - before));
    Serial.print(F(" s)"));
  }
  Serial.println();
  return true;
}

bool syncTimeViaNTP() {
  Serial.println(F("🌐 Synchronizacja czasu z serwerem NTP (pool.ntp.org)..."));
  sendAT("AT+CNTP=\"pool.ntp.org\",8", 2000, false);
  const char* r = sendAT("AT+CNTP", 3000, false);
  if (strstr(r, "ERROR")) {
    Serial.println(F("⚠️ AT+CNTP zwróciło ERROR."));
    return false;
  }
  const char* p = readUntil("+CNTP:", 10000);
  if (!p) {
    Serial.println(F("⚠️ Brak odpowiedzi +CNTP (timeout)."));
    return false;
  }
  int code = atoi(p + 6);
  if (code != 1) {
    Serial.print(F("⚠️ NTP nieudane, kod: "));
    Serial.println(code);
    return false;
  }
  return true;
}

bool syncClock() {
  bool ok = syncEspRtcFromModem();
  if (!ok && ensurePdpActive(NULL)) {
    if (syncTimeViaNTP()) ok = syncEspRtcFromModem();
  }
  if (ok) lastTimeSyncEpoch = time(NULL);
  return ok;
}

bool getEspDateTimeOffset(char* outBuffer, size_t maxLen) {
  time_t now = time(NULL);
  if (now < 1704067200) return false;

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  const char* tzStr = (timeinfo.tm_isdst > 0) ? "+02:00" : "+01:00";
  snprintf(outBuffer, maxLen, "%04d-%02d-%02dT%02d:%02d:%02d%s",
           timeinfo.tm_year + 1900,
           timeinfo.tm_mon + 1,
           timeinfo.tm_mday,
           timeinfo.tm_hour,
           timeinfo.tm_min,
           timeinfo.tm_sec,
           tzStr);
  return true;
}

// ============================================================================
// REJESTRACJA W SIECI LTE (JAWNY PODGLĄD STANU CO 2.5 S)
// ============================================================================
bool connectToNetwork(uint16_t* outNetTimeS, int16_t* outLastStat) {
  uint32_t netStart = millis();
  Serial.println(F("\n🌐 Łączenie z siecią LTE..."));
  sendAT("AT+CFUN=1", 2000, false);

  char apnCmd[64];
  snprintf(apnCmd, sizeof(apnCmd), "AT+CGDCONT=1,\"IP\",\"%s\"", MODEM_APN);
  sendAT(apnCmd, 1500, false);

  Serial.println(F("📡 Oczekiwanie na rejestrację LTE (max 60 s):"));

  uint32_t start = millis();
  int stat = -1;

  while (millis() - start < NET_REG_TIMEOUT_MS) {
    uint32_t elapsedS = (millis() - start) / 1000;
    stat = ceregStatus();
    if (outLastStat) *outLastStat = (int16_t)stat;

    Serial.print(F("  ["));
    Serial.print(elapsedS);
    Serial.print(F("s] CEREG: "));
    Serial.print(stat);
    Serial.print(F(" -> "));
    Serial.println(ceregStatusDesc(stat));

    if (stat == 1 || stat == 5) {
      if (outNetTimeS) *outNetTimeS = (uint16_t)((millis() - netStart) / 1000);
      Serial.print(F("✅ Zarejestrowano w sieci LTE w "));
      Serial.print((millis() - netStart) / 1000);
      Serial.println(F(" s!"));
      return true;
    }

    delaySafe(2500);
  }

  if (outNetTimeS) *outNetTimeS = (uint16_t)((millis() - netStart) / 1000);
  Serial.println(F("❌ Nie zarejestrowano w sieci w zadanym czasie 60 s."));
  return false;
}

void ensureNetwork(uint16_t* outNetTimeS, int16_t* outLastStat) {
  sendAT("AT+CFUN=1", 1000, false);

  char apnCmd[64];
  snprintf(apnCmd, sizeof(apnCmd), "AT+CGDCONT=1,\"IP\",\"%s\"", MODEM_APN);
  sendAT(apnCmd, 1000, false);

  if (isRegistered()) {
    networkReady = true;
    if (outNetTimeS) *outNetTimeS = 0;
    if (outLastStat) *outLastStat = (int16_t)ceregStatus();
    return;
  }
  networkReady = connectToNetwork(outNetTimeS, outLastStat);
}

// ============================================================================
// TRANSMISJA HTTP POST (TIMEOUTY 60 S)
// ============================================================================

bool parseIpContext0(const char* resp, char* outIp, size_t maxLen) {
  if (!resp || !outIp || maxLen < 8) return false;

  const char* p = strstr(resp, "+CNACT: 0,1,");
  if (!p) p = strstr(resp, "+CNACT: 0, 1,");
  if (!p) p = strstr(resp, "0,1,");
  if (!p) p = strstr(resp, "0, 1,");
  if (!p) return false;

  const char* q1 = strchr(p, '"');
  if (!q1) return false;
  const char* q2 = strchr(q1 + 1, '"');
  if (!q2) return false;

  size_t len = q2 - (q1 + 1);
  if (len < 7 || len >= maxLen) return false;

  char temp[32];
  strncpy(temp, q1 + 1, len);
  temp[len] = '\0';

  if (strcmp(temp, "0.0.0.0") == 0 || strchr(temp, '.') == NULL) return false;

  strncpy(outIp, temp, maxLen);
  return true;
}

bool ensurePdpActive(uint8_t* outErr) {
  char currentIp[32] = {0};

  const char* chk = sendAT("AT+CNACT?", 1200, false);
  if (parseIpContext0(chk, currentIp, sizeof(currentIp))) {
    Serial.print(F("✅ Sesja pakietowa aktywna. IP: "));
    Serial.println(currentIp);
    return true;
  }

  Serial.println(F("🌐 Konfiguracja i aktywacja sesji pakietowej (CNACT)..."));

  sendAT("AT+CGATT=1", 2000, false);
  sendAT("AT+CNACT=0,0", 1500, false);
  delaySafe(100);

  char cncfgCmd[64];
  snprintf(cncfgCmd, sizeof(cncfgCmd), "AT+CNCFG=0,1,\"%s\"", MODEM_APN);
  sendAT(cncfgCmd, 1500, false);

  const char* rAct = sendAT("AT+CNACT=0,1", 6000, false);
  if (strstr(rAct, "ERROR")) {
    if (outErr) *outErr = ERR_PDP_ACT_FAILED;
  }

  Serial.print(F("⏳ Oczekiwanie na dzierżawę IP (max 60 s): "));
  uint32_t start = millis();
  uint32_t lastPrint = millis();
  while (millis() - start < 60000UL) {
    delaySafe(1000);
    const char* r = sendAT("AT+CNACT?", 1000, false);
    if (parseIpContext0(r, currentIp, sizeof(currentIp))) {
      Serial.print(F("\n✅ Uzyskano adres IP: "));
      Serial.println(currentIp);
      return true;
    }
    if (millis() - lastPrint >= 2000) {
      uint32_t el = (millis() - start) / 1000;
      Serial.print(F("["));
      Serial.print(el);
      Serial.print(F("s]"));
      lastPrint = millis();
    } else {
      Serial.print('.');
    }
  }

  Serial.println(F("\n❌ BŁĄD: Operator nie nadał adresu IP w czasie 60 s."));
  if (outErr && *outErr == ERR_OK) *outErr = ERR_IP_LEASE_TIMEOUT;
  return false;
}

void readHttpResponseBody(int dataLen, char* outMsg, size_t maxLen) {
  if (dataLen <= 0 || !outMsg || maxLen == 0) {
    if (outMsg) outMsg[0] = '\0';
    return;
  }
  markStep(ST_SHREAD);
  int toRead = (dataLen > 48) ? 48 : dataLen;
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+SHREAD=0,%d", toRead);
  const char* r = sendAT(cmd, 1500, true);

  const char* p = strstr(r, "+SHREAD:");
  if (p) {
    const char* lineEnd = strstr(p, "\r\n");
    if (lineEnd) {
      const char* bodyStart = lineEnd + 2;
      const char* okPos = strstr(bodyStart, "\r\nOK");
      size_t bodyBytes = okPos ? (size_t)(okPos - bodyStart) : strlen(bodyStart);
      if (bodyBytes >= maxLen) bodyBytes = maxLen - 1;
      char rawTemp[64] = {0};
      strncpy(rawTemp, bodyStart, bodyBytes);
      rawTemp[bodyBytes] = '\0';
      sanitizeErrorMessage(outMsg, rawTemp, maxLen);
      Serial.print(F("💬 Odpowiedź serwera: "));
      Serial.println(outMsg);
      return;
    }
  }
  outMsg[0] = '\0';
}

bool sendHttpPost(const char* jsonBody, uint8_t* outErr, uint16_t* outReqDurationMs, int16_t* outDetail, char* outMsg, size_t maxMsgLen) {
  markStep(ST_PDP);
  if (!ensurePdpActive(outErr)) {
    Serial.println(F("🛑 Brak adresu IP — pomijam żądanie HTTP POST."));
    return false;
  }

  size_t bodyLen = strlen(jsonBody);
  Serial.println(F("\n🚀 Przygotowanie wysyłki HTTP POST..."));

  markStep(ST_HTTP_CONF);
  
  const char* stateResp = sendAT("AT+SHSTATE?", 1000, false);
  if (strstr(stateResp, "+SHSTATE: 1")) {
    Serial.println(F("ℹ️ Wykryto wiszącą sesję HTTP — wymuszam SHDISC..."));
    sendAT("AT+SHDISC", 1500, true);
    delaySafe(200);
  } else {
    sendAT("AT+SHDISC", 800, false);
  }

  sendAT("AT+SHSSL=0", 800, true);

  char confUrl[128];
  snprintf(confUrl, sizeof(confUrl), "AT+SHCONF=\"URL\",\"%s\"", API_HOST);
  sendAT(confUrl, 1000, true);
  char bodyLenCmd[40];
  snprintf(bodyLenCmd, sizeof(bodyLenCmd), "AT+SHCONF=\"BODYLEN\",%d", HTTP_BODYLEN);
  sendAT(bodyLenCmd, 800, true);
  sendAT("AT+SHCONF=\"HEADERLEN\",350", 800, true);

  Serial.println(F("⏳ Zestawianie połączenia TCP z tunelem/serwerem (timeout 60 s)..."));

  markStep(ST_SHCONN);
  const char* connRes = sendAT("AT+SHCONN", 60000, true);
  if (!strstr(connRes, "OK")) {
    Serial.println(F("❌ Błąd połączenia z serwerem (AT+SHCONN po 60 s)."));
    if (outErr) *outErr = ERR_TCP_CONN_FAILED;
    sendAT("AT+SHDISC", 1000, false);
    return false;
  }

  markStep(ST_HTTP_CONF);
  sendAT("AT+SHCHEAD", 800, true);
  sendAT("AT+SHAHEAD=\"Content-Type\",\"application/json\"", 800, true);
  sendAT("AT+SHAHEAD=\"Connection\",\"close\"", 800, true);
  sendAT("AT+SHAHEAD=\"Bypass-Tunnel-Reminder\",\"true\"", 800, true);

  sendAT("ATE0", 400, false);

  markStep(ST_SHBOD);
  char bodCmd[32];
  snprintf(bodCmd, sizeof(bodCmd), "AT+SHBOD=%u,5000", (unsigned)bodyLen);
  while (Serial2.available()) Serial2.read();
  Serial.print(F(" > "));
  Serial.println(bodCmd);
  Serial2.println(bodCmd);

  uint32_t tStart = millis();
  bool promptReady = false;
  while (millis() - tStart < 4000) {
    while (Serial2.available()) {
      if (Serial2.read() == '>') {
        promptReady = true;
        break;
      }
    }
    if (promptReady) break;
    delay(2);
    yield();
    esp_task_wdt_reset();
  }

  if (!promptReady) {
    Serial.println(F("❌ Brak promptu '>' dla bufora modemu."));
    if (outErr) *outErr = ERR_HTTP_PROMPT_TIMEOUT;
    sendAT("AT+SHDISC", 800, false);
    return false;
  }

  Serial.println(F(" > [Wysyłanie payloadu JSON do modemu...]"));
  Serial2.print(jsonBody);

  tStart = millis();
  bool bodyAccepted = false;
  char slidingBuf[64] = {0};
  size_t sIdx = 0;

  while (millis() - tStart < 6000UL) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (sIdx < sizeof(slidingBuf) - 1) {
        slidingBuf[sIdx++] = c;
        slidingBuf[sIdx] = '\0';
      } else {
        memmove(slidingBuf, slidingBuf + 1, sizeof(slidingBuf) - 2);
        slidingBuf[sizeof(slidingBuf) - 2] = c;
        slidingBuf[sizeof(slidingBuf) - 1] = '\0';
      }
    }

    if (strstr(slidingBuf, "OK")) {
      bodyAccepted = true;
      break;
    }
    delay(5);
    yield();
    esp_task_wdt_reset();
  }

  if (!bodyAccepted) {
    Serial.println(F("❌ Modem nie zatwierdził treści JSON."));
    if (outErr) *outErr = ERR_HTTP_BODY_REJECTED;
    sendAT("AT+SHDISC", 800, false);
    return false;
  }

  markStep(ST_SHREQ);
  char reqCmd[128];
  snprintf(reqCmd, sizeof(reqCmd), "AT+SHREQ=\"%s\",3", API_PATH);
  while (Serial2.available()) Serial2.read();
  Serial.print(F(" > "));
  Serial.println(reqCmd);

  uint32_t reqStart = millis();
  uint32_t lastReqPrint = millis();
  Serial2.println(reqCmd);

  int httpStatus = 0;
  int responseDataLen = 0;
  char respBuf[128] = {0};
  size_t respLen = 0;

  while (millis() - reqStart < 45000UL) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (respLen < sizeof(respBuf) - 1) {
        respBuf[respLen++] = c;
        respBuf[respLen] = '\0';
      }
    }

    const char* pStatus = strstr(respBuf, "+SHREQ:");
    if (pStatus && strstr(pStatus, "\r\n")) {
      if (outReqDurationMs) *outReqDurationMs = (uint16_t)(millis() - reqStart);
      pStatus += 7;
      while (*pStatus == ' ' || *pStatus == '"') pStatus++;

      int t1 = 0, t2 = 0;
      const char* c1 = strchr(pStatus, ',');
      if (c1) {
        t1 = atoi(pStatus);
        const char* c2 = strchr(c1 + 1, ',');
        if (c2) {
          t1 = atoi(c1 + 1);
          t2 = atoi(c2 + 1);
        } else {
          t2 = atoi(c1 + 1);
        }
      } else {
        t1 = atoi(pStatus);
      }

      if (t1 >= 100 && t1 < 600) {
        httpStatus = t1;
        responseDataLen = t2;
      }
      break;
    }
    if (strstr(respBuf, "ERROR\r\n")) break;

    if (millis() - lastReqPrint >= 2000) {
      uint32_t el = (millis() - reqStart) / 1000;
      Serial.print(F("[req:"));
      Serial.print(el);
      Serial.print(F("s]"));
      lastReqPrint = millis();
    }

    delay(10);
    yield();
    esp_task_wdt_reset();
  }

  Serial.println();
  Serial.print(F(" < "));
  Serial.println(respBuf[0] ? respBuf : "(brak odpowiedzi statusu / TIMEOUT 45s)");
  Serial.print(F("📥 Kod odpowiedzi HTTP API: "));
  Serial.println(httpStatus);

  if (outDetail) *outDetail = (int16_t)httpStatus;

  if (httpStatus >= 400 && responseDataLen > 0 && outMsg) {
    readHttpResponseBody(responseDataLen, outMsg, maxMsgLen);
  }

  sendAT("AT+SHDISC", 1000, false);

  if (httpStatus >= 200 && httpStatus < 300) {
    return true;
  } else if (httpStatus == 0) {
    if (outErr) *outErr = ERR_HTTP_REQ_TIMEOUT;
    return false;
  } else {
    if (outErr) *outErr = ERR_HTTP_STATUS_NOT_2XX;
    return false;
  }
}

// ============================================================================
// DEEP SLEEP
// ============================================================================

void goToSleep(uint32_t seconds) {
  markStep(ST_SLEEP);
  Serial.print(F("\n💤 Usypianie procesora ESP32 w tryb Deep Sleep na "));
  Serial.print(seconds);
  Serial.println(F(" s..."));
  Serial.flush();
  delay(100);

  gpio_hold_dis((gpio_num_t)HX711_SCK);
  pinMode(HX711_SCK, OUTPUT);
  digitalWrite(HX711_SCK, HIGH);
  gpio_hold_en((gpio_num_t)HX711_SCK);
  gpio_deep_sleep_hold_en();

  esp_sleep_enable_timer_wakeup((uint64_t)seconds * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(50);

#if defined(ARDUINO_USB_CDC_ON_BOOT) && (ARDUINO_USB_CDC_ON_BOOT == 1)
  Serial.setTxTimeoutMs(0);
#endif

  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();

  initPersistState();
  bootResetReason = esp_reset_reason();
  markStep(ST_BOOT);

  setupWatchdog();
}

// ============================================================================
// STATYCZNE BUFORY BSS
// ============================================================================
static char rawStr[1280];
static size_t rawLen = 0;
static char jsonPayload[JSON_BUF_SIZE];

struct RawCycleData {
  float waga;
  float dsTemp;
  long hxRaw;
  long hxNoise;
  float chipTemp;
  float vbat;
  const char* vbatSrc;
  int csq;
  uint16_t activeS;
  uint16_t netS;
  uint8_t cycleErr;
  int16_t cycleDetail;
};
static RawCycleData rawData;

void rawAppend(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

void rawAppend(const char* fmt, ...) {
  size_t room = sizeof(rawStr) - rawLen;
  if (room <= 1) return;
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(rawStr + rawLen, room, fmt, ap);
  va_end(ap);
  if (w < 0) return;
  rawLen += ((size_t)w < room) ? (size_t)w : (room - 1);
}

const char* shortCodeName(uint8_t code) {
  const char* n = getStatusCodeName(code);
  return (strncmp(n, "ERR_", 4) == 0) ? n + 4 : n;
}

const char* detailLabel(uint8_t code) {
  switch (code) {
    case ERR_HTTP_STATUS_NOT_2XX:
    case ERR_HTTP_REQ_TIMEOUT:    return "http";
    case ERR_NET_REG_TIMEOUT:     return "cereg";
    case ERR_BATTERY_CRITICAL:    return "Vx100";
    case ERR_HARDWARE_CRASH:
    case ERR_WATCHDOG_RESET:
    case ERR_BATTERY_BROWNOUT:    return "krok";
    case ERR_JSON_SERIALIZE_FAIL: return "bajty";
    default:                      return "det";
  }
}

void rawAppendError(uint8_t i) {
  const ErrorLogEntry& e = ps.errQueue[i];
  bool hasTs = (strlen(e.timestamp) >= 19 && strcmp(e.timestamp, "null") != 0);
  rawAppend(" [#%lu %.14s %s(%u) %s:%d",
            (unsigned long)e.report_id,
            hasTs ? e.timestamp + 5 : "--",
            shortCodeName(e.code), (unsigned)e.code,
            detailLabel(e.code), (int)e.detail);
  if (e.msg[0]) rawAppend(" '%s'", e.msg);
  if (e.vbat_v > 0.5f) rawAppend(" %.2fV", (double)e.vbat_v);
  if (e.csq >= 0 && e.csq < 99) rawAppend(" csq:%d", (int)e.csq);
  rawAppend("]");
}

void buildRawStr(uint8_t droppedErrs) {
  const RawCycleData& d = rawData;
  rawLen = 0;
  rawStr[0] = '\0';

  rawAppend("v%s | #%u | %.2fkg (raw:%ld, szum:%ld) | T_zew:%.2fC (chip:%.1fC) | VBAT:%.2fV(%s) CSQ:%d | CZAS:%us (net:%us, http:%ums)",
            FIRMWARE_VERSION, msgCounter, (double)d.waga, d.hxRaw, d.hxNoise,
            (double)d.dsTemp, (double)d.chipTemp,
            (double)d.vbat, d.vbatSrc, d.csq,
            (unsigned)d.activeS, (unsigned)d.netS, (unsigned)lastHttpTimeMs);

  rawAppend(" | RST:%s(%d)@%s", getResetReasonName(bootResetReason), (int)bootResetReason,
            getStepName(stepAtReset));

  if (d.cycleErr == ERR_OK) {
    rawAppend(" | TEN CYKL:OK");
  } else {
    rawAppend(" | TEN CYKL:%s(%u) %s:%d", shortCodeName(d.cycleErr), (unsigned)d.cycleErr,
              detailLabel(d.cycleErr), (int)d.cycleDetail);
  }

  rawAppend(" | BLEDY(%u):", (unsigned)ps.errCount);
  if (ps.errCount == 0) rawAppend(" brak");
  for (uint8_t i = 0; i < ps.errCount; i++) rawAppendError(i);

  if (droppedErrs > 0) {
    rawAppend(" | error_logs przyciete o %u najstarszych (pelne dane wyzej)", (unsigned)droppedErrs);
  }
}

// ============================================================================
// LOOP GŁÓWNY
// ============================================================================

void loop() {
  uint32_t cycleStartMs = millis();
  uint8_t currentCycleError = ERR_OK;
  int16_t currentCycleErrorDetail = 0;
  char currentCycleErrorMsg[36] = {0};
  uint16_t currentNetTimeS = 0;
  uint16_t currentHttpTimeMs = 0;
  int16_t lastCeregStat = -1;

  markStep(ST_BOOT);

  prefs.begin("waga", false);
  uint32_t lastReportId = prefs.getUInt("counter", 0);
  msgCounter = lastReportId + 1;
  prefs.putUInt("counter", msgCounter);
  prefs.end();

  Serial.println(F("\n========================================"));
  Serial.print(F("📻 Waga Pasieczna (SIM7080G + DS18B20) v"));
  Serial.print(FIRMWARE_VERSION);
  Serial.print(F(" | RAPORT #"));
  Serial.println(msgCounter);
  Serial.print(F("↩️ Reset: "));
  Serial.print(getResetReasonName(bootResetReason));
  Serial.print(F("("));
  Serial.print((int)bootResetReason);
  Serial.print(F("), ostatni krok poprzedniego cyklu: "));
  Serial.println(getStepName(stepAtReset));
  Serial.println(F("========================================"));

  checkHardwareResetReason(lastReportId);

  char timeVal[35] = "";
  bool hasValidTime = getEspDateTimeOffset(timeVal, sizeof(timeVal));
  float vbatVal = 0.0f;
  const char* vbatSrc = "-";
  bool batteryChecked = false;
  int csqVal = 99;

#if VBAT_SOURCE_MAX17048
  markStep(ST_BATTERY);
  vbatVal = sanitizeFloat(readMax17048Voltage(), 0.0f);
  if (vbatVal > 0.5f) {
    batteryChecked = true;
    vbatSrc = "MAX";
    if (batteryBlocksTx(vbatVal, msgCounter, timeVal)) {
      goToSleep(TIME_TO_SLEEP_LOWBATT);
    }
  } else {
    Serial.println(F("⚠️ [BAT] MAX17048 nie odpowiada — decyzja na podstawie AT+CBC z modemu."));
  }
#endif

  // 1. Pomiar temperatury z czujnika DS18B20
  float dsTemp = zmierzTemperatureDS18B20();

  // 2. Pomiar temperatury wewnętrznej procesora ESP32
  float chipTemp = sanitizeFloat(temperatureRead(), 0.0f);

  // 3. Pomiar tensometru HX711 ze średnią uciętą (16 próbek)
  markStep(ST_HX711);
  long hxRaw = 0;
  long hxNoise = 0;
  float waga = zmierzWage(&hxRaw, &hxNoise, &currentCycleError);
  if (isnan(waga)) {
    waga = -99.99f;
    if (currentCycleError == ERR_OK) currentCycleError = ERR_HX711_NAN;
  }

  // 4. Rozruch modemu SIM7080G (baza v7.5)
  markStep(ST_MODEM_ON);
  bool modemStarted = ensureModemOn();
  Serial.println(F("🔄 Inicjalizacja modemu..."));

  markStep(ST_MODEM_CFG);
  bool modemOk = false;
  if (!modemStarted) {
    Serial.println(F("❌ Modem nie odpowiedział na komendy AT."));
    currentCycleError = ERR_MODEM_NO_AT;
    strcpy(currentCycleErrorMsg, "NO_AT");
  } else {
    modemOk = configureModem();
    if (!modemOk) {
      Serial.println(F("❌ Błąd konfiguracji modemu."));
      currentCycleError = ERR_MODEM_NO_AT;
      strcpy(currentCycleErrorMsg, "NO_AT");
    } else {
      Serial.println(F("✅ Modem zsynchronizowany."));
      sendAT("AT+GSN", 1000, true);

      if (!batteryChecked) {
        markStep(ST_BATTERY);
        if (ps.lowBatt) {
          sendAT("AT+CFUN=0", 10000, false);
        }
        vbatVal = getBatteryVoltage(true);
        vbatSrc = (vbatVal > 0.5f) ? "CBC" : "-";
        if (batteryBlocksTx(vbatVal, msgCounter, timeVal)) {
          turnOffModem();
          goToSleep(TIME_TO_SLEEP_LOWBATT);
        }
      } else {
        getBatteryVoltage(true);
      }

      markStep(ST_RADIO_CFG);
      ensureRadioConfig();

      markStep(ST_NET_REG);
      ensureNetwork(&currentNetTimeS, &lastCeregStat);
      if (!networkReady && currentCycleError == ERR_OK) {
        currentCycleError = ERR_NET_REG_TIMEOUT;
        currentCycleErrorDetail = lastCeregStat;
        strcpy(currentCycleErrorMsg, "NO_REG");
      }
    }
  }

  // 5. Zegar RTC ESP32
  markStep(ST_TIME_SYNC);
  if (networkReady && needTimeSync()) {
    Serial.println(isEspTimeValid() ? F("⏳ Okresowa resynchronizacja zegara ESP32...")
                                    : F("⏳ Zegar ESP32 wymaga synchronizacji..."));
    if (!syncClock()) {
      Serial.println(F("⚠️ Synchronizacja zegara nieudana."));
    }
  }

  hasValidTime = getEspDateTimeOffset(timeVal, sizeof(timeVal));
  if (hasValidTime) {
    Serial.print(F("🕒 Znacznik czasu ESP32 RTC: "));
    Serial.println(timeVal);
  } else {
    Serial.println(F("⚠️ Czas niedostępny — wysyłam null"));
  }

  // 6. Odczyt CSQ z trójstopniową kaskadą LTE
  if (networkReady) {
    csqVal = checkSignalQuality();
  } else {
    csqVal = 99;
  }

  uint16_t activeTimeS = (uint16_t)((millis() - cycleStartMs) / 1000);

  // 7. Budowanie struktury JSON
  markStep(ST_JSON);
  rawData.waga = waga;
  rawData.dsTemp = dsTemp;
  rawData.hxRaw = hxRaw;
  rawData.hxNoise = hxNoise;
  rawData.chipTemp = chipTemp;
  rawData.vbat = vbatVal;
  rawData.vbatSrc = vbatSrc;
  rawData.csq = csqVal;
  rawData.activeS = activeTimeS;
  rawData.netS = currentNetTimeS;
  rawData.cycleErr = currentCycleError;
  rawData.cycleDetail = currentCycleErrorDetail;
  buildRawStr(0);

  JsonDocument doc;
  doc["scale_id"] = SCALE_ID;
  if (hasValidTime) {
    doc["timestamp"] = timeVal;
  } else {
    doc["timestamp"] = nullptr;
  }
  doc["report_id"] = msgCounter;
  doc["weight_kg"] = waga;

  if (!isnan(dsTemp)) {
    doc["temp_c"] = dsTemp;
  } else {
    doc["temp_c"] = nullptr;
  }

  doc["raw"] = rawStr;

  JsonObject loraObj = doc["lora"].to<JsonObject>();
  loraObj["rssi"] = 0.0;
  loraObj["snr"] = 0.00;

  JsonObject hubObj = doc["hub"].to<JsonObject>();
  hubObj["vbat_v"] = vbatVal;
  hubObj["csq"] = csqVal;
  hubObj["chip_temp_c"] = chipTemp;

  if (!isnan(dsTemp)) {
    hubObj["temp_ds18b20_c"] = dsTemp;
  } else {
    hubObj["temp_ds18b20_c"] = nullptr;
  }

  hubObj["active_time_s"] = activeTimeS;
  hubObj["net_time_s"] = currentNetTimeS;
  hubObj["http_time_ms"] = lastHttpTimeMs;

  JsonObject diagObj = doc["sensor_diag"].to<JsonObject>();
  diagObj["hx_raw"] = hxRaw;
  diagObj["hx_noise"] = hxNoise;

  JsonArray errArray = doc["error_logs"].to<JsonArray>();
  for (uint8_t i = 0; i < ps.errCount; i++) {
    const ErrorLogEntry& e = ps.errQueue[i];
    JsonObject errItem = errArray.add<JsonObject>();
    errItem["report_id"] = e.report_id;
    if (strcmp(e.timestamp, "null") == 0 || strlen(e.timestamp) == 0) {
      errItem["timestamp"] = nullptr;
    } else {
      errItem["timestamp"] = e.timestamp;
    }
    errItem["code"] = e.code;
    errItem["name"] = getStatusCodeName(e.code);
    errItem["detail"] = e.detail;
    errItem["msg"] = e.msg;
    errItem["vbat_v"] = sanitizeFloat(e.vbat_v, 0.0f);
    errItem["csq"] = e.csq;
  }

  // WERSJA 8.3: Twarde czyszczenie pamięci błędów (Punkt 3)
  uint8_t droppedErrs = 0;
  while (measureJson(doc) >= sizeof(jsonPayload) && errArray.size() > 0) {
    errArray.remove(0); // Usunięcie z lokalnego bufora JSON
    droppedErrs++;
    
    // Bezpowrotne usunięcie błędu ze sprzętowej czarnej skrzynki (RTC)
    if (ps.errCount > 0) {
      memmove(&ps.errQueue[0], &ps.errQueue[1], sizeof(ErrorLogEntry) * (ps.errCount - 1));
      ps.errCount--;
    }

    buildRawStr(droppedErrs);
    doc["raw"] = rawStr;
  }
  
  if (droppedErrs > 0) {
    Serial.print(F("✂️ Payload za duży — usunięto najstarszych wpisów z JSON oraz trwałej pamięci RTC: "));
    Serial.println(droppedErrs);
  }

  size_t needed = measureJson(doc);
  if (needed >= sizeof(jsonPayload)) {
    size_t excess = needed - (sizeof(jsonPayload) - 1);
    size_t len = strlen(rawStr);
    if (len > excess + 3) {
      size_t cut = len - excess - 3;
      memcpy(rawStr + cut, "...", 4);
      doc["raw"] = rawStr;
      Serial.println(F("✂️ Skrócono koniec pola raw, żeby zmieścić payload."));
    }
  }

  size_t bytesWritten = 0;
  if (!doc.overflowed()) {
    bytesWritten = serializeJson(doc, jsonPayload, sizeof(jsonPayload));
  }

  if (doc.overflowed() || bytesWritten == 0 || bytesWritten >= sizeof(jsonPayload)) {
    Serial.println(F("❌ BŁĄD KRYTYCZNY: Serializacja JSON nie powiodła się!"));
    currentCycleError = ERR_JSON_SERIALIZE_FAIL;
    currentCycleErrorDetail = (int16_t)bytesWritten;
  } else {
    Serial.println(F("\n📦 SFORMATOWANY PAYLOAD HTTP (ArduinoJson):"));
    Serial.println(jsonPayload);
    Serial.print(F("Rozmiar: "));
    Serial.print(bytesWritten);
    Serial.println(F(" bajtów."));
    Serial.println(F("----------------------------------------"));
  }

  // 8. Wysłanie danych do API
  bool httpSuccess = false;

  if (networkReady && currentCycleError != ERR_JSON_SERIALIZE_FAIL) {
    if (sendHttpPost(jsonPayload, &currentCycleError, &currentHttpTimeMs, &currentCycleErrorDetail, currentCycleErrorMsg, sizeof(currentCycleErrorMsg))) {
      Serial.println(F("🎉 Sukces: Dane zapisane na serwerze API!"));
      httpSuccess = true;
      ps.errCount = 0; // Wyczyszczenie całej kolejki po poprawnym wysłaniu
      lastHttpTimeMs = currentHttpTimeMs;
    } else {
      Serial.println(F("⚠️ Błąd: Nie udało się dostarczyć danych do API."));
    }
  } else {
    Serial.println(F("📵 Brak sieci LTE lub błąd JSON — wysyłka wstrzymana."));
  }

  if (!httpSuccess) {
    if (currentCycleError == ERR_OK) currentCycleError = ERR_TCP_CONN_FAILED;
    recordError(msgCounter, timeVal, currentCycleError, currentCycleErrorDetail, currentCycleErrorMsg, vbatVal, csqVal);
    Serial.print(F("💾 Zarejestrowano błąd w kolejce RTC RAM: #"));
    Serial.print(msgCounter);
    Serial.print(F(" -> "));
    Serial.print(getStatusCodeName(currentCycleError));
    Serial.print(F(" (detail: "));
    Serial.print(currentCycleErrorDetail);
    if (strlen(currentCycleErrorMsg) > 0) {
      Serial.print(F(", msg: '"));
      Serial.print(currentCycleErrorMsg);
      Serial.print(F("'"));
    }
    Serial.println(F(")"));
  }

  // 9. Wyłączenie modemu i Deep Sleep
  turnOffModem();
  goToSleep(TIME_TO_SLEEP);
}