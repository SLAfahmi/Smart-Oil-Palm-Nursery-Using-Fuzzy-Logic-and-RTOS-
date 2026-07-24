#include <Arduino.h>
#include <BH1750.h>
#include <DHT.h>
#include <SPIFFS.h>
#include <Wire.h>

#include <ArduinoJson.h>
#include <RTClib.h>
#include <WebServer.h>
#include <WiFi.h>

// ==========================================
// WIFI
// ==========================================

const char *apSsid = "SmartNursery-AP";
const char *apPassword = "12345678";

// ==========================================
// PIN
// ==========================================

const int soilPin[5] = {32, 34, 35, 33, 36};

#define DHT_PIN 4
#define DHT_TYPE DHT22

#define RELAY_MAIN 13
// #define RELAY_POMPA 13 //udah gadipakai
const int relayPin[5] = {15, 18, 19, 5, 27};

// RELAY ACTIVE LOW
#define RELAY_ON LOW
#define RELAY_OFF HIGH

// ==========================================
// SENSOR & WEBSERVER
// ==========================================

DHT dht(DHT_PIN, DHT_TYPE);
BH1750 lightMeter;
RTC_DS3231 rtc;
WebServer server(80);

// ==========================================
// TASK HANDLE
// ==========================================

TaskHandle_t fuzzyHandle;
TaskHandle_t irrigationHandle[5];
TaskHandle_t webHandle;
TaskHandle_t manualControlHandle;
TaskHandle_t scheduleHandle;
SemaphoreHandle_t zonaMutex;
SemaphoreHandle_t maxConcurrentIrrigationSemaphore;

// ==========================================
// VARIABLE
// ==========================================

uint32_t lastWateringTime[5] = {
    0}; // Menyimpan waktu terakhir disiram (Unix time / fallback millis)
const uint32_t COOLDOWN_SECONDS =
    4 * 60 * 60; // Cooldown setelah penyiraman ke penyiraman berikutnya 4 jam
bool rtcAvailable = false;

uint32_t getCurrentTimeSeconds() {
  if (rtcAvailable) {
    DateTime now = rtc.now();
    return now.unixtime();
  }
  return millis() / 1000UL;
}

struct Zona {
  float soil;
  float duration;
  int mode; // 0=Off, 1=Short, 2=Medium, 3=Long
};

Zona zona[5];

float temperature;
float humidity;
float lightLevel;

// Count of active irrigation tasks
volatile int activeIrrigations = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// 0 = Auto, 1 = Manual, 2 = Schedule
int systemMode = 0;
bool manualRelayMain = false;
bool manualRelayState[5] = {false};

struct Schedule {
  bool active;
  uint8_t hour;
  uint8_t minute;
  uint32_t duration;
};
#define MAX_SCHEDULES 3
Schedule zoneSchedules[5][MAX_SCHEDULES];
int lastScheduledMinute = -1;

// ==========================================
// CONFIGURATIONS (WEB)
// ==========================================
uint32_t cfgSensorInterval = 10; // detik
int cfgManualMode = 0;           // 0 = Toggle, 1 = Timer
uint32_t cfgManualDur = 10;      // detik (durasi timer manual)

// Timer untuk manual mode
uint32_t manualTimerOff[5] = {0};
uint32_t manualStartTime[5] = {0};

void updateMainRelayState() {
  bool anyOn = false;
  for (int i = 0; i < 5; i++) {
    if (manualRelayState[i]) {
      anyOn = true;
      break;
    }
  }
  manualRelayMain = anyOn;
  digitalWrite(RELAY_MAIN, manualRelayMain ? RELAY_ON : RELAY_OFF);
  // digitalWrite(RELAY_POMPA, manualRelayMain ? RELAY_ON : RELAY_OFF);
}

// ==========================================
// LOG BUFFER
// ==========================================
struct LogEntry {
  char timestamp[20];
  float temp;
  float hum;
  float light;
  float soil[5];
  int mode[5];
  float duration[5];
  int sysMode;
};
#define MAX_LOG_ENTRIES 100
LogEntry logBuffer[MAX_LOG_ENTRIES];
int logHead = 0;
int logCount = 0;

void addLogEntry() {
  DateTime now = rtc.now();
  sprintf(logBuffer[logHead].timestamp, "%04d/%02d/%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(), now.hour(), now.minute(),
          now.second());
  logBuffer[logHead].temp = temperature;
  logBuffer[logHead].hum = humidity;
  logBuffer[logHead].light = lightLevel;
  for (int i = 0; i < 5; i++) {
    logBuffer[logHead].soil[i] = zona[i].soil;
    logBuffer[logHead].mode[i] = zona[i].mode;
    logBuffer[logHead].duration[i] = zona[i].duration;
  }
  logBuffer[logHead].sysMode = systemMode;
  logHead = (logHead + 1) % MAX_LOG_ENTRIES;
  if (logCount < MAX_LOG_ENTRIES)
    logCount++;
}

// ==========================================
// FUZZY FUNCTION (UMUM)
// ==========================================

float fungsiMin(float a, float b) { return (a < b) ? a : b; }

float fungsiMin3(float a, float b, float c) {
  return fungsiMin(a, fungsiMin(b, c));
}

float fungsiMax(float a, float b) { return (a > b) ? a : b; }

// ==========================================
// INPUT 1: KELEMBABAN TANAH (%)
// NOTE: kelembabanTanah di sini memakai mapping yang UDAH GWEH GANTI
// (ADC kecil = basah, ADC besar = kering).
// 0% = kering, 100% = basah
// "basah/kering" ditangani di dalam fungsi keanggotaan,
// bukan di mapping ADC -> lihat SensorTask.
// ==========================================
// ==========================================
// FUNGSI KEANGGOTAAN FUZZY (BERDASARKAN Y)
// ==========================================

// 1. Himpunan Kering (Y rendah)
float f_Kering(float y) {
  if (y <= 12.0)
    return 1.0;
  if (y > 12.0 && y < 20.0)
    return (20.0 - y) / (20.0 - 12.0);
  return 0.0;
}

// 2. Himpunan Lembab (Y sedang)
float f_Lembab(float y) {
  if (y <= 12.0 || y >= 30.0)
    return 0.0;
  if (y > 12.0 && y <= 21.0)
    return (y - 12.0) / (21.0 - 12.0);
  if (y > 21.0 && y < 30.0)
    return (30.0 - y) / (30.0 - 21.0);
  return 0.0;
}

// 3. Himpunan Basah (Y tinggi)
float f_Basah(float y) {
  if (y <= 22.0)
    return 0.0;
  if (y > 22.0 && y < 30.0)
    return (y - 22.0) / (30.0 - 22.0);
  return 1.0;
}

// ==========================================
// INPUT 2: SUHU UDARA (°C)
// ==========================================
float f_Dingin(float x) {
  if (x <= 22)
    return 1.0;
  if (x > 22 && x < 26)
    return (26.0 - x) / (26.0 - 22.0);
  return 0.0;
}

float f_Normal(float x) {
  if (x <= 24 || x >= 32)
    return 0.0;
  if (x > 24 && x <= 28)
    return (x - 24.0) / (28.0 - 24.0);
  if (x > 28 && x < 32)
    return (32.0 - x) / (32.0 - 28.0);
  return 0.0;
}

float f_Panas(float x) {
  if (x <= 30)
    return 0.0;
  if (x > 30 && x < 35)
    return (x - 30.0) / (35.0 - 30.0);
  return 1.0;
}

// ==========================================
// INPUT 3: CAHAYA (Lux) -> Batas Max 65.000 Lux
// ==========================================
float f_Redup(float x) {
  if (x <= 5000)
    return 1.0;
  if (x > 5000 && x < 25000)
    return (25000.0 - x) / (25000.0 - 5000.0);
  return 0.0;
}

// Himpunan Sedang: Segitiga (Transisi antara Redup & Terang)
float f_Sedang(float x) {
  if (x <= 5000 || x >= 65000)
    return 0.0;
  if (x > 5000 && x <= 35000)
    return (x - 5000.0) / (35000.0 - 5000.0);
  if (x > 35000 && x < 65000)
    return (65000.0 - x) / (65000.0 - 35000.0);
  return 0.0;
}

// Himpunan Terang: Trapesium kanan
float f_Terang(float x) {
  if (x <= 45000)
    return 0.0;
  if (x > 45000 && x < 65000)
    return (x - 45000.0) / (65000.0 - 45000.0);
  return 1.0;
}

// ==========================================
// OUTPUT: DURASI PENYIRAMAN (Detik)
// ==========================================
float f_Mati(float z) {
  if (z <= 0)
    return 1.0;
  if (z > 0 && z < 10)
    return (10.0 - z) / (10.0 - 0.0);
  return 0.0;
}

float f_Pendek(float z) {
  if (z <= 10 || z >= 90)
    return 0.0;
  if (z > 10 && z <= 50)
    return (z - 10.0) / (50.0 - 10.0);
  if (z > 50 && z < 90)
    return (90.0 - z) / (90.0 - 50.0);
  return 0.0;
}

float f_SedangOut(float z) {
  if (z <= 70 || z >= 190)
    return 0.0;
  if (z > 70 && z <= 130)
    return (z - 70.0) / (130.0 - 70.0);
  if (z > 130 && z < 190)
    return (190.0 - z) / (190.0 - 130.0);
  return 0.0;
}

float f_Lama(float z) {
  if (z <= 170)
    return 0.0;
  if (z > 170 && z < 222)
    return (z - 170.0) / (222.0 - 170.0);
  if (z >= 222)
    return 1.0;
  return 0.0;
}

// ==========================================
// FUZZY MAMDANI PROSES
// ==========================================
float prosesFuzzyMamdani(float kelembaban, float suhu, float cahaya) {
  float mu_Kering = f_Kering(kelembaban);
  float mu_Lembab = f_Lembab(kelembaban);
  float mu_Basah = f_Basah(kelembaban);

  float mu_Dingin = f_Dingin(suhu);
  float mu_Normal = f_Normal(suhu);
  float mu_Panas = f_Panas(suhu);

  float mu_Redup = f_Redup(cahaya);
  float mu_Sedang = f_Sedang(cahaya);
  float mu_Terang = f_Terang(cahaya);

  float r_Mati = 0.0;
  float r_Pendek = 0.0;
  float r_Sedang = 0.0;
  float r_Lama = 0.0;

  // --- Rule Base ---
  // Urutan argumen fungsiMin3 = (Kelembaban, Suhu, Cahaya)

  // OUTPUT: SEDANG
  r_Sedang = fungsiMax(r_Sedang, fungsiMin3(mu_Kering, mu_Dingin, mu_Redup));
  r_Sedang = fungsiMax(r_Sedang, fungsiMin3(mu_Kering, mu_Dingin, mu_Sedang));
  r_Sedang = fungsiMax(r_Sedang, fungsiMin3(mu_Kering, mu_Normal, mu_Redup));
  r_Sedang = fungsiMax(r_Sedang, fungsiMin3(mu_Lembab, mu_Normal, mu_Terang));
  r_Sedang = fungsiMax(r_Sedang, fungsiMin3(mu_Lembab, mu_Panas, mu_Redup));
  r_Sedang = fungsiMax(r_Sedang, fungsiMin3(mu_Lembab, mu_Panas, mu_Sedang));
  r_Sedang = fungsiMax(r_Sedang, fungsiMin3(mu_Lembab, mu_Panas, mu_Terang));

  // OUTPUT: LAMA
  r_Lama = fungsiMax(r_Lama, fungsiMin3(mu_Kering, mu_Dingin, mu_Terang));
  r_Lama = fungsiMax(r_Lama, fungsiMin3(mu_Kering, mu_Normal, mu_Sedang));
  r_Lama = fungsiMax(r_Lama, fungsiMin3(mu_Kering, mu_Normal, mu_Terang));
  r_Lama = fungsiMax(r_Lama, fungsiMin3(mu_Kering, mu_Panas, mu_Redup));
  r_Lama = fungsiMax(r_Lama, fungsiMin3(mu_Kering, mu_Panas, mu_Sedang));
  r_Lama = fungsiMax(r_Lama, fungsiMin3(mu_Kering, mu_Panas, mu_Terang));

  // OUTPUT: PENDEK
  r_Pendek = fungsiMax(r_Pendek, fungsiMin3(mu_Lembab, mu_Dingin, mu_Redup));
  r_Pendek = fungsiMax(r_Pendek, fungsiMin3(mu_Lembab, mu_Dingin, mu_Sedang));
  r_Pendek = fungsiMax(r_Pendek, fungsiMin3(mu_Lembab, mu_Dingin, mu_Terang));
  r_Pendek = fungsiMax(r_Pendek, fungsiMin3(mu_Lembab, mu_Normal, mu_Redup));
  r_Pendek = fungsiMax(r_Pendek, fungsiMin3(mu_Lembab, mu_Normal, mu_Sedang));

  // OUTPUT: MATI (tanah BASAH -> pompa tidak menyiram sama sekali)
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Dingin, mu_Redup));
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Dingin, mu_Sedang));
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Dingin, mu_Terang));
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Normal, mu_Redup));
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Normal, mu_Sedang));
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Normal, mu_Terang));
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Panas, mu_Redup));
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Panas, mu_Sedang));
  r_Mati = fungsiMax(r_Mati, fungsiMin3(mu_Basah, mu_Panas, mu_Terang));

  // --- Defuzzifikasi (Metode Centroid) ---
  float pembilang = 0.0;
  float penyebut = 0.0;

  // Evaluasi dari 0 hingga 222 detik
  for (float z = 0; z <= 222; z += 1.0) {
    float m = fungsiMin(f_Mati(z), r_Mati);
    float p = fungsiMin(f_Pendek(z), r_Pendek);
    float s = fungsiMin(f_SedangOut(z), r_Sedang);
    float l = fungsiMin(f_Lama(z), r_Lama);

    float total = fungsiMax(fungsiMax(m, p), fungsiMax(s, l));

    pembilang += z * total;
    penyebut += total;
  }

  if (penyebut == 0.0)
    return 0.0;

  float hasil = pembilang / penyebut;

  // safety guard: jika hasil sangat kecil mendekati nol, bulatkan ke 0.
  if (hasil < 2.0)
    return 0.0;

  return hasil;
}
// ==========================================
// WEBSERVER HANDLERS
// ==========================================

void serveSpiffs(const char *path, const char *contentType) {
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
  } else {
    server.send(404, "text/plain", "File Not Found");
  }
}

void handleRoot() { serveSpiffs("/index.html", "text/html"); }

void handleCSS() { serveSpiffs("/style.css", "text/css"); }

void handleData() {
  JsonDocument doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["lightLevel"] = lightLevel;
  doc["systemMode"] = systemMode;
  doc["relayMain"] = manualRelayMain;

  JsonArray arr = doc["zona"].to<JsonArray>();
  if (xSemaphoreTake(zonaMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (int i = 0; i < 5; i++) {
      JsonObject z = arr.add<JsonObject>();
      z["id"] = i + 1;
      z["soil"] = zona[i].soil;
      z["mode"] = zona[i].mode;
      z["duration"] = zona[i].duration;
      z["relayState"] = manualRelayState[i];
    }
    xSemaphoreGive(zonaMutex);
  }

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// handle untuk mengubah mode system biar tidak tabrakan sama handle relay dan
// fuzzy
void handleOverride() {
  if (server.hasArg("mode")) {
    systemMode = server.arg("mode").toInt();

    // Reset all relays when toggling manual mode to prevent stuck relays
    manualRelayMain = false;
    digitalWrite(RELAY_MAIN, RELAY_OFF);
    // digitalWrite(RELAY_POMPA, RELAY_OFF);

    for (int i = 0; i < 5; i++) {
      manualRelayState[i] = false;
      manualTimerOff[i] = 0;
      digitalWrite(relayPin[i], RELAY_OFF);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleRelay() {
  if (systemMode == 1 && server.hasArg("relay")) {
    String relay = server.arg("relay");
    bool isTimer = (cfgManualMode == 1);
    uint32_t durMs = cfgManualDur * 1000;

    if (relay == "main") {
      manualRelayMain = !manualRelayMain;
      digitalWrite(RELAY_MAIN, manualRelayMain ? RELAY_ON : RELAY_OFF);
      // digitalWrite(RELAY_POMPA, manualRelayMain ? RELAY_ON : RELAY_OFF);
    } else {
      int idx = relay.toInt();
      if (idx >= 0 && idx < 5) {
        if (!manualRelayState[idx]) {
          // Cek maksimal 3 sub aktif
          int activeCount = 0;
          for (int i = 0; i < 5; i++) {
            if (manualRelayState[i])
              activeCount++;
          }
          if (activeCount >= 3) {
            server.send(400, "text/plain",
                        "MAKSIMAL 3 ZONA AKTIF | GAKUAT AKI NYA");
            return;
          }

          // Turn ON
          manualRelayState[idx] = true;
          manualStartTime[idx] = millis();
          if (isTimer) {
            manualTimerOff[idx] = millis() + durMs;
            zona[idx].duration = cfgManualDur;
          }
          digitalWrite(relayPin[idx], RELAY_ON);
        } else {
          // Turn OFF
          manualRelayState[idx] = false;
          digitalWrite(relayPin[idx], RELAY_OFF);
          manualTimerOff[idx] = 0;

          // Log final duration
          zona[idx].duration = (millis() - manualStartTime[idx]) / 1000.0;
          addLogEntry();
          zona[idx].duration = 0;
        }
        updateMainRelayState();
      }
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleConfigGet() {
  JsonDocument doc;
  doc["sensorInterval"] = cfgSensorInterval;
  doc["manualMode"] = cfgManualMode;
  doc["manualDur"] = cfgManualDur;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleConfigPost() {
  if (server.hasArg("plain")) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if (!error) {
      if (doc["sensorInterval"].is<uint32_t>())
        cfgSensorInterval = doc["sensorInterval"];
      if (doc["manualMode"].is<int>())
        cfgManualMode = doc["manualMode"];
      if (doc["manualDur"].is<uint32_t>())
        cfgManualDur = doc["manualDur"];
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Bad Request");
}

void saveSchedule() {
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < 5; i++) {
    JsonObject z = arr.add<JsonObject>();
    JsonArray sArr = z["schedules"].to<JsonArray>();
    for (int j = 0; j < MAX_SCHEDULES; j++) {
      JsonObject sObj = sArr.add<JsonObject>();
      sObj["active"] = zoneSchedules[i][j].active;
      sObj["hour"] = zoneSchedules[i][j].hour;
      sObj["minute"] = zoneSchedules[i][j].minute;
      sObj["duration"] = zoneSchedules[i][j].duration;
    }
  }
  File file = SPIFFS.open("/schedule.json", "w");
  if (file) {
    serializeJson(doc, file);
    file.close();
  }
}

void loadSchedule() {
  if (SPIFFS.exists("/schedule.json")) {
    File file = SPIFFS.open("/schedule.json", "r");
    if (file) {
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, file);
      if (!error) {
        JsonArray arr = doc.as<JsonArray>();
        for (int i = 0; i < 5 && i < arr.size(); i++) {
          JsonObject z = arr[i];
          JsonArray sArr = z["schedules"];
          for (int j = 0; j < MAX_SCHEDULES && j < sArr.size(); j++) {
            JsonObject sObj = sArr[j];
            zoneSchedules[i][j].active = sObj["active"] | false;
            zoneSchedules[i][j].hour = sObj["hour"] | 0;
            zoneSchedules[i][j].minute = sObj["minute"] | 0;
            zoneSchedules[i][j].duration = sObj["duration"] | 10;
          }
        }
      }
      file.close();
    }
  }
}

void handleScheduleGet() {
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < 5; i++) {
    JsonObject z = arr.add<JsonObject>();
    z["id"] = i + 1;
    JsonArray sArr = z["schedules"].to<JsonArray>();
    for (int j = 0; j < MAX_SCHEDULES; j++) {
      JsonObject sObj = sArr.add<JsonObject>();
      sObj["active"] = zoneSchedules[i][j].active;
      sObj["hour"] = zoneSchedules[i][j].hour;
      sObj["minute"] = zoneSchedules[i][j].minute;
      sObj["duration"] = zoneSchedules[i][j].duration;
    }
  }
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSchedulePost() {
  if (server.hasArg("plain")) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if (!error && doc.is<JsonArray>()) {
      JsonArray arr = doc.as<JsonArray>();
      for (int i = 0; i < 5 && i < arr.size(); i++) {
        JsonObject z = arr[i];
        JsonArray sArr = z["schedules"];
        for (int j = 0; j < MAX_SCHEDULES && j < sArr.size(); j++) {
          JsonObject sObj = sArr[j];
          zoneSchedules[i][j].active = sObj["active"] | false;
          zoneSchedules[i][j].hour = sObj["hour"] | 0;
          zoneSchedules[i][j].minute = sObj["minute"] | 0;
          zoneSchedules[i][j].duration = sObj["duration"] | 10;
        }
      }
      saveSchedule();
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Bad Request");
}

void handleLogGet() {
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();

  int startIdx = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
  for (int i = 0; i < logCount; i++) {
    int idx = (startIdx + i) % MAX_LOG_ENTRIES;
    JsonObject obj = arr.add<JsonObject>();
    obj["timestamp"] = logBuffer[idx].timestamp;
    obj["temp"] = logBuffer[idx].temp;
    obj["hum"] = logBuffer[idx].hum;
    obj["light"] = logBuffer[idx].light;

    JsonArray sArr = obj["soil"].to<JsonArray>();
    JsonArray mArr = obj["mode"].to<JsonArray>();
    JsonArray dArr = obj["duration"].to<JsonArray>();
    for (int j = 0; j < 5; j++) {
      sArr.add(logBuffer[idx].soil[j]);
      mArr.add(logBuffer[idx].mode[j]);
      dArr.add(logBuffer[idx].duration[j]);
    }
    obj["sysMode"] = logBuffer[idx].sysMode;
  }

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// ==========================================
// SENSOR TASK
// ==========================================

void SensorTask(void *pv) {
  // Tunggu 500ms agar semua task & handle lain selesai dibuat oleh setup()
  vTaskDelay(pdMS_TO_TICKS(500));

  while (1) {
    if (xSemaphoreTake(zonaMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      for (int i = 0; i < 5; i++) {
        int adc = analogRead(soilPin[i]);

        // 1. Regresi Linier (ADC ke Moisture Analyzer Y)
        float a = 41.7453982;
        float b = -0.012665492;
        float Y = a + (b * adc);

        // 2. Mapping nilai Y (10 - 35) ke persentase (0 - 100%)
        // Menggunakan rumus proporsi: (Y - Y_min) * (100 / (Y_max - Y_min))
        float mappedSoil = (Y - 10.0) * (100.0 / 25.0);

        // 3. Batasi nilai agar tetap di rentang 0 - 100
        zona[i].soil = constrain(mappedSoil, 0.0f, 100.0f);
      }
      xSemaphoreGive(zonaMutex);
    }

    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    lightLevel = lightMeter.readLightLevel();

    if (isnan(temperature))
      temperature = 00.0;
    if (isnan(humidity))
      humidity = 00.0;

    static uint32_t lastHourlyLog = 0;
    uint32_t currentSec = getCurrentTimeSeconds();

    // Log per jam (3600 detik)
    if (lastHourlyLog == 0 || (currentSec - lastHourlyLog >= 3600)) {
      addLogEntry();
      lastHourlyLog = currentSec;
    }

    if (systemMode == 0 && activeIrrigations == 0) {
      xTaskNotifyGive(fuzzyHandle);
    }

    vTaskDelay(pdMS_TO_TICKS(cfgSensorInterval * 1000));
  }
}

// ==========================================
// FUZZY TASK
// ==========================================

void FuzzyTask(void *pv) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (systemMode != 0) {
      continue; // Skip fuzzy control if not in Auto mode
    }

    if (xSemaphoreTake(zonaMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      uint32_t currentTime = getCurrentTimeSeconds();

      for (int i = 0; i < 5; i++) {
        float durasi =
            prosesFuzzyMamdani(zona[i].soil, temperature, lightLevel);

        // Cek cooldown: Jika belum lewat 4 jam, abaikan hasil fuzzy (set ke 0)
        if (lastWateringTime[i] != 0 &&
            (currentTime - lastWateringTime[i]) < COOLDOWN_SECONDS) {
          durasi = 0;
        }

        zona[i].duration = durasi;
        if (durasi > 0) {
          zona[i].mode = 4; // 4 = Queued
          xTaskNotifyGive(irrigationHandle[i]);
        } else {
          zona[i].mode = 0;
          zona[i].duration = 0;
        }
      }
      xSemaphoreGive(zonaMutex);
    }
  }
}

// ==========================================
// IRRIGATION TASK
// ==========================================

void IrrigationTask(void *pv) {
  int id = (int)(uintptr_t)pv;

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (systemMode == 1) {
      continue;
    }

    // Tunggu giliran jika sudah ada 3 sub yang aktif
    xSemaphoreTake(maxConcurrentIrrigationSemaphore, portMAX_DELAY);

    // Cek ulang mode jika user mengubah ke manual saat task sedang di antrian
    // (queue)
    if (systemMode == 1) {
      if (xSemaphoreTake(zonaMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        zona[id].mode = 0;
        zona[id].duration = 0;
        xSemaphoreGive(zonaMutex);
      }
      xSemaphoreGive(maxConcurrentIrrigationSemaphore);
      continue;
    }

    // Set mode aktual (Active) setelah berhasil keluar dari antrian
    if (xSemaphoreTake(zonaMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (systemMode == 2) {
        zona[id].mode = 3; // Schedule mode
      } else {
        if (zona[id].duration <= 20)
          zona[id].mode = 1;
        else if (zona[id].duration <= 40)
          zona[id].mode = 2;
        else
          zona[id].mode = 3;
      }
      xSemaphoreGive(zonaMutex);
    }

    portENTER_CRITICAL(&mux);
    activeIrrigations++;
    portEXIT_CRITICAL(&mux);

    digitalWrite(relayPin[id], RELAY_ON);
    digitalWrite(RELAY_MAIN, RELAY_ON);
    // digitalWrite(RELAY_POMPA, RELAY_ON);

    vTaskDelay(pdMS_TO_TICKS((uint32_t)(zona[id].duration * 1000)));

    if (systemMode != 1) {
      digitalWrite(relayPin[id], RELAY_OFF);
    }

    addLogEntry();

    // Catat waktu selesai penyiraman untuk cooldown
    lastWateringTime[id] = getCurrentTimeSeconds();

    // Reset mode so UI shows IDLE when no zone is actively irrigating
    if (xSemaphoreTake(zonaMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      zona[id].mode = 0;
      zona[id].duration = 0;
      xSemaphoreGive(zonaMutex);
    }

    portENTER_CRITICAL(&mux);
    activeIrrigations--;
    if (activeIrrigations == 0 && systemMode != 1) {
      digitalWrite(RELAY_MAIN, RELAY_OFF);
      // digitalWrite(RELAY_POMPA, RELAY_OFF);
    }
    portEXIT_CRITICAL(&mux);

    xSemaphoreGive(maxConcurrentIrrigationSemaphore);
  }
}

// ==========================================
// WEB TASK
// ==========================================

void WebTask(void *pv) {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/style.css", HTTP_GET, handleCSS);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/override", HTTP_POST, handleOverride);
  server.on("/api/relay", HTTP_POST, handleRelay);
  server.on("/api/config", HTTP_GET, handleConfigGet);
  server.on("/api/config", HTTP_POST, handleConfigPost);
  server.on("/api/schedule", HTTP_GET, handleScheduleGet);
  server.on("/api/schedule", HTTP_POST, handleSchedulePost);
  server.on("/api/log", HTTP_GET, handleLogGet);

  server.begin();

  while (1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==========================================
// MANUAL CONTROL TASK
// ==========================================

void ManualControlTask(void *pv) {
  while (1) {
    if (systemMode == 1 && cfgManualMode == 1) {
      uint32_t now = millis();
      bool changed = false;

      for (int i = 0; i < 5; i++) {
        if (manualRelayState[i] && manualTimerOff[i] > 0 &&
            now >= manualTimerOff[i]) {
          manualRelayState[i] = false;
          digitalWrite(relayPin[i], RELAY_OFF);
          manualTimerOff[i] = 0;

          zona[i].duration = (now - manualStartTime[i]) / 1000.0;
          addLogEntry();
          zona[i].duration = 0;

          changed = true;
        }
      }

      if (changed) {
        updateMainRelayState();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ==========================================
// SCHEDULE TASK
// ==========================================

void ScheduleTask(void *pv) {
  while (1) {
    if (systemMode == 2) {
      DateTime now = rtc.now();
      int currentHour = now.hour();
      int currentMinute = now.minute();

      // Cek setiap menit saja
      if (lastScheduledMinute != currentMinute) {
        lastScheduledMinute = currentMinute;

        if (xSemaphoreTake(zonaMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
          for (int i = 0; i < 5; i++) {
            for (int j = 0; j < MAX_SCHEDULES; j++) {
              if (zoneSchedules[i][j].active &&
                  zoneSchedules[i][j].hour == currentHour &&
                  zoneSchedules[i][j].minute == currentMinute) {
                // Cek jika soil moisture <= 60 (tidak terlalu basah)
                if (zona[i].soil <= 60.0) {
                  zona[i].duration = zoneSchedules[i][j].duration;
                  // Masukkan ke dalam antrian terlebih dahulu (mode 4)
                  zona[i].mode = 4;
                  xTaskNotifyGive(irrigationHandle[i]);
                }
                break; // Jika salah satu jadwal cocok, tidak usah cek jadwal
                       // lain untuk menit ini
              }
            }
          }
          xSemaphoreGive(zonaMutex);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ==========================================
// SETUP
// ==========================================

void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin(true)) {
    Serial.println("Error mounting SPIFFS");
  }

  pinMode(RELAY_MAIN, OUTPUT);
  digitalWrite(RELAY_MAIN, RELAY_OFF);
  // pinMode(RELAY_POMPA, OUTPUT);
  // digitalWrite(RELAY_POMPA, RELAY_OFF);
  for (int i = 0; i < 5; i++) {
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i], RELAY_OFF);
  }

  Wire.begin();
  dht.begin();

  if (!lightMeter.begin()) {
    Serial.println("Gagal menemukan sensor BH1750!");
  }

  if (!rtc.begin()) {
    rtcAvailable = false;
    Serial.println(
        "Gagal menemukan RTC DS3231! Menggunakan millis() untuk cooldown");
  } else {
    rtcAvailable = true;
    if (rtc.lostPower()) {
      Serial.println(
          "RTC kehilangan daya, mengatur waktu ke waktu kompilasi...");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  // Mode WIFI_AP saja (hanya pakai WiFi dari ESP)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid, apPassword);

  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  zonaMutex = xSemaphoreCreateMutex();
  maxConcurrentIrrigationSemaphore = xSemaphoreCreateCounting(3, 3);

  xTaskCreate(SensorTask, "Sensor", 4096, NULL, 5, NULL);
  xTaskCreate(FuzzyTask, "Fuzzy", 4096, NULL, 4, &fuzzyHandle);

  for (int i = 0; i < 5; i++) {
    xTaskCreate(IrrigationTask, "Irrigation", 2048, (void *)(uintptr_t)i, 3,
                &irrigationHandle[i]);
  }

  // Stack untuk WebTask sedikit lebih besar agar aman menangani HTTP request
  // dan JSON
  xTaskCreate(WebTask, "Web", 8192, NULL, 2, &webHandle);

  // Task khusus untuk mode manual (menggantikan void loop)
  xTaskCreate(ManualControlTask, "ManualControl", 2048, NULL, 1,
              &manualControlHandle);

  // Task untuk penjadwalan
  xTaskCreate(ScheduleTask, "Schedule", 2048, NULL, 1, &scheduleHandle);

  // Load jadwal dari SPIFFS
  loadSchedule();

  Serial.println("Smart Nursery Siap");
}

void loop() {
  
  vTaskDelete(NULL);
}
