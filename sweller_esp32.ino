#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <Adafruit_NeoPixel.h>
#include <SD.h>
#include <SPI.h>
#include "time.h"
#include <driver/i2s.h>
#include <esp_task_wdt.h>   // FIX #11: Watchdog

// ============================================================================
// HARDWARE & PIN CONFIGURATION
// ============================================================================
#define CURRENT_VERSION   15   // FIX #10: #define not const int

#define NEO_PIN           48

#define SD_CS             10
#define SPI_MOSI          11
#define SPI_SCK           12
#define SPI_MISO          13

#define I2S_WS            15
#define I2S_SCK           16
#define I2S_SD            17
#define I2S_PORT          I2S_NUM_0

#define SAMPLE_RATE       16000
#define I2S_BUFFER_SIZE   1024

#define WDT_TIMEOUT_SEC   60   // FIX #11: reboot if stuck for 60 s

Adafruit_NeoPixel pixel(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
// NETWORK & URL CONFIGURATION
// ============================================================================
const char* ssid     = "LKGJOEY";
const char* password = "krmangalamlkgjoey";

IPAddress local_IP(192, 168, 1, 152);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);

const char* piServerIP        = "http://192.168.1.3:5000";
const char* githubVersionUrl  = "https://raw.githubusercontent.com/05Ashish/sweller_esp32/main/version.txt";
const char* githubFirmwareUrl = "https://raw.githubusercontent.com/05Ashish/sweller_esp32/main/build/esp32.esp32.esp32s3/sweller_esp32.ino.bin";

const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 19800; // IST: GMT+5:30
const int   daylightOffset_sec = 0;

// ============================================================================
// STATE — persisted to SD so reboots mid-class don't double-record
// FIX #1 & #5: track both period AND the calendar day it was recorded
// ============================================================================
#define STATE_FILE "/state.txt"

int  lastRecordedPeriod = -1;
int  lastRecordedDay    = -1;  // tm_yday (0–365)
bool sdReady            = false;

// ============================================================================
// VISUAL STATUS
// ============================================================================
void setColor(int r, int g, int b) {
  pixel.setPixelColor(0, pixel.Color(g, r, b));
  pixel.show();
}

// ============================================================================
// STATE PERSISTENCE  (FIX #1 & #5)
// ============================================================================
void loadState() {
  if (!sdReady) return;
  File f = SD.open(STATE_FILE, FILE_READ);
  if (!f) return;
  String line  = f.readStringUntil('\n');
  f.close();
  int comma = line.indexOf(',');
  if (comma < 0) return;
  lastRecordedPeriod = line.substring(0, comma).toInt();
  lastRecordedDay    = line.substring(comma + 1).toInt();
  Serial.printf("[STATE] Loaded: period=%d  day=%d\n",
                lastRecordedPeriod, lastRecordedDay);
}

// Write-then-copy so a power-cut during write can't corrupt the state file.
void saveState() {
  if (!sdReady) return;
  SD.remove("/state.tmp");
  File f = SD.open("/state.tmp", FILE_WRITE);
  if (!f) return;
  f.printf("%d,%d\n", lastRecordedPeriod, lastRecordedDay);
  f.close();
  SD.remove(STATE_FILE);
  File src = SD.open("/state.tmp", FILE_READ);
  File dst = SD.open(STATE_FILE,   FILE_WRITE);
  if (src && dst) { while (src.available()) dst.write(src.read()); }
  if (src) src.close();
  if (dst) dst.close();
  SD.remove("/state.tmp");
}

// ============================================================================
// AUDIO & I2S
// ============================================================================
void initI2S() {
  i2s_config_t i2s_config = {
    .mode               = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate        = SAMPLE_RATE,
    .bits_per_sample    = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format     = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags   = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count      = 8,
    .dma_buf_len        = 1024,
    .use_apll           = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk         = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void writeWavHeader(File& file, uint32_t dataSize) {
  // channels=1 mono | bitsPerSample=16
  // blockAlign = channels × (bitsPerSample/8) = 1×2 = 2
  // byteRate   = sampleRate × blockAlign       = 16000×2 = 32000
  byte     header[44];
  uint32_t fileSize = dataSize + 36;
  uint32_t byteRate = SAMPLE_RATE * 2;

  memcpy(header,      "RIFF", 4);
  header[4]  = (byte)(fileSize        & 0xFF);
  header[5]  = (byte)((fileSize >> 8) & 0xFF);
  header[6]  = (byte)((fileSize >>16) & 0xFF);
  header[7]  = (byte)((fileSize >>24) & 0xFF);
  memcpy(header + 8,  "WAVE", 4);
  memcpy(header + 12, "fmt ", 4);
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  header[20] = 1;  header[21] = 0;   // PCM
  header[22] = 1;  header[23] = 0;   // mono
  header[24] = (byte)(SAMPLE_RATE        & 0xFF);
  header[25] = (byte)((SAMPLE_RATE >> 8) & 0xFF);
  header[26] = (byte)((SAMPLE_RATE >>16) & 0xFF);
  header[27] = (byte)((SAMPLE_RATE >>24) & 0xFF);
  header[28] = (byte)(byteRate        & 0xFF);
  header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >>16) & 0xFF);
  header[31] = (byte)((byteRate >>24) & 0xFF);
  header[32] = 2;  header[33] = 0;   // blockAlign
  header[34] = 16; header[35] = 0;   // bitsPerSample
  memcpy(header + 36, "data", 4);
  header[40] = (byte)(dataSize        & 0xFF);
  header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >>16) & 0xFF);
  header[43] = (byte)((dataSize >>24) & 0xFF);

  file.seek(0);
  file.write(header, 44);
}

// FIX #4: guard against zero / near-zero duration recordings
void recordClassAudio(const String& fileName, int durationSeconds) {
  if (durationSeconds < 10) {
    Serial.printf("[REC] Only %d seconds remain — skipping period.\n",
                  durationSeconds);
    return;
  }

  String filePath  = "/pending/" + fileName;
  File   audioFile = SD.open(filePath, FILE_WRITE);
  if (!audioFile) {
    Serial.println("[REC] ERROR: Could not open file for writing.");
    return;
  }

  Serial.printf("[REC] '%s'  duration=%dm%ds\n",
                fileName.c_str(), durationSeconds / 60, durationSeconds % 60);
  setColor(50, 0, 0); // RED = recording

  audioFile.seek(44); // reserve WAV header space

  size_t        bytesRead;
  int16_t       i2sData[I2S_BUFFER_SIZE / 2];
  uint32_t      totalDataBytes   = 0;
  unsigned long startMillis      = millis();
  unsigned long recordDurationMs = (unsigned long)durationSeconds * 1000UL;

  while (millis() - startMillis < recordDurationMs) {
    esp_task_wdt_reset(); // FIX #11: keep watchdog happy during long recording
    // FIX #11: 1 s timeout instead of portMAX_DELAY so WDT can be reached
    i2s_read(I2S_PORT, &i2sData, I2S_BUFFER_SIZE, &bytesRead, pdMS_TO_TICKS(1000));
    if (bytesRead > 0) {
      audioFile.write((const byte*)i2sData, bytesRead);
      totalDataBytes += bytesRead;
    }
    yield();
  }

  writeWavHeader(audioFile, totalDataBytes);
  audioFile.close();
  Serial.printf("[REC] Done — %u bytes written.\n", totalDataBytes);
  setColor(0, 50, 0); // GREEN = done
}

// ============================================================================
// NETWORK & NTP
// ============================================================================
void syncTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.print("[NTP] Waiting for time sync");
  struct tm     timeinfo;
  unsigned long start = millis();
  while (!getLocalTime(&timeinfo) && millis() - start < 10000) {
    Serial.print(".");
    delay(500);
  }
  if (getLocalTime(&timeinfo)) {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.printf("\n[NTP] Synced: %s\n", buf);
  } else {
    Serial.println("\n[NTP] WARNING: Could not sync time!");
  }
}

void keepWiFiAlive() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("[WiFi] Reconnecting...");
  WiFi.config(local_IP, gateway, subnet, primaryDNS);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    setColor(50, 20, 0); delay(250);
    setColor(0,  0,  0); delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Reconnected.");
    syncTime();
  } else {
    Serial.println("[WiFi] Reconnect failed — will retry next loop.");
  }
}

// ============================================================================
// SD SPACE ENFORCEMENT
// FIX #2: skip the file currently being uploaded (skipFile param)
// FIX #3: always close dir handle
// ============================================================================
void enforceSDSpace(const String& skipFile = "") {
  while (true) {
    uint64_t total     = SD.totalBytes();
    uint64_t used      = SD.usedBytes();
    float    freeSpace = 100.0f * (float)(total - used) / (float)total;
    if (freeSpace >= 5.0f) break;

    File   dir      = SD.open("/pending");
    String toDelete = "";

    while (true) {
      File f = dir.openNextFile();
      if (!f) break;
      if (!f.isDirectory()) {
        String name = String(f.name());
        f.close();
        if (name != skipFile) { toDelete = "/pending/" + name; break; }
      } else {
        f.close();
      }
    }
    dir.close(); // FIX #3

    if (toDelete == "") {
      Serial.println("[SD] Cannot free space — all files protected.");
      break;
    }
    Serial.println("[SD] Purging: " + toDelete);
    SD.remove(toDelete);
  }
}

// ============================================================================
// UPLOAD
// FIX #7: removed jitter — it blocked the main loop and could cause missed periods
// FIX #8: per-file retry counter stored on SD; give up after MAX_UPLOAD_RETRIES
// FIX #3: dir handle always closed
// ============================================================================
#define MAX_UPLOAD_RETRIES 3

void processPendingUploads() {
  if (WiFi.status() != WL_CONNECTED) return;

  File dir = SD.open("/pending");
  if (!dir) return;

  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    if (f.isDirectory()) { f.close(); continue; }

    String fileName = String(f.name());
    String filePath = "/pending/" + fileName;
    String retryKey = "/retries/" + fileName + ".cnt";

    // FIX #8: read retry count
    int retryCount = 0;
    {
      File rc = SD.open(retryKey, FILE_READ);
      if (rc) { retryCount = rc.readString().toInt(); rc.close(); }
    }

    if (retryCount >= MAX_UPLOAD_RETRIES) {
      Serial.printf("[UPLOAD] '%s' exceeded retry limit — deleting.\n",
                    fileName.c_str());
      f.close();
      SD.remove(filePath);
      SD.remove(retryKey);
      continue;
    }

    Serial.printf("[UPLOAD] '%s' attempt %d/%d\n",
                  fileName.c_str(), retryCount + 1, MAX_UPLOAD_RETRIES);

    enforceSDSpace(fileName); // FIX #2: protect current file from purge

    HTTPClient http;
    http.setTimeout(15000);
    http.begin(String(piServerIP) + "/upload");
    http.addHeader("X-Filename", fileName);

    int httpCode = http.sendRequest("POST", &f, f.size());
    f.close();
    http.end();

    if (httpCode == 200) {
      Serial.printf("[UPLOAD] '%s' OK.\n", fileName.c_str());
      SD.remove(filePath);
      SD.remove(retryKey);
    } else {
      Serial.printf("[UPLOAD] '%s' failed HTTP %d.\n", fileName.c_str(), httpCode);
      SD.remove(retryKey);
      File rc = SD.open(retryKey, FILE_WRITE);
      if (rc) { rc.printf("%d\n", retryCount + 1); rc.close(); }
    }
    yield();
  }

  dir.close(); // FIX #3
}

// ============================================================================
// GITHUB OTA
// ============================================================================
String getServerVersion() {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(5000);
  http.begin(client, githubVersionUrl);
  String version = "";
  if (http.GET() == 200) { version = http.getString(); version.trim(); }
  http.end();
  return version;
}

void checkOTA() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("\n[OTA] Checking for updates...");
  setColor(0, 0, 50);

  String serverVersion = getServerVersion();
  if (serverVersion == "") {
    Serial.println("[OTA] Could not read version.");
    setColor(0, 0, 0);
    return;
  }

  int newVersion = serverVersion.toInt();
  Serial.printf("[OTA] Current=%d  Server=%d\n", CURRENT_VERSION, newVersion);

  if (newVersion > CURRENT_VERSION) {
    Serial.println("[OTA] Downloading...");
    setColor(25, 0, 25);

    WiFiClientSecure client;
    client.setInsecure();
    httpUpdate.rebootOnUpdate(false);

    t_httpUpdate_return ret = httpUpdate.update(client, githubFirmwareUrl);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("[OTA] Failed: %s\n", httpUpdate.getLastErrorString().c_str());
        setColor(50, 0, 0); delay(2000); setColor(0, 0, 0);
        break;
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("[OTA] Server says no update.");
        setColor(0, 0, 0);
        break;
      case HTTP_UPDATE_OK:
        Serial.println("[OTA] Success — rebooting.");
        setColor(0, 50, 0); delay(1500);
        ESP.restart();
        break;
    }
  } else {
    Serial.println("[OTA] Up to date.");
    setColor(0, 0, 0);
  }
}

// ============================================================================
// TIMETABLE
// FIX #5: reset lastRecordedPeriod when the calendar day changes
// FIX #4: skip if < 10 seconds remain in the period
// FIX #1: saveState() before recording so a mid-record crash is safe
// ============================================================================
void checkTimetableAndRecord() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;

  // FIX #5: new calendar day — allow all periods to record again
  if (timeinfo.tm_yday != lastRecordedDay) {
    Serial.printf("[TIME] New day (yday=%d). Resetting period tracker.\n",
                  timeinfo.tm_yday);
    lastRecordedPeriod = -1;
    lastRecordedDay    = timeinfo.tm_yday;
    saveState();
  }

  int nowMins       = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int currentPeriod = -1;
  int remainingMins = 0;

  if      (nowMins >= 505 && nowMins < 540) { currentPeriod = 1;   remainingMins = 540 - nowMins; }
  else if (nowMins >= 540 && nowMins < 575) { currentPeriod = 2;   remainingMins = 575 - nowMins; }
  else if (nowMins >= 575 && nowMins < 610) { currentPeriod = 3;   remainingMins = 610 - nowMins; }
  else if (nowMins >= 640 && nowMins < 675) { currentPeriod = 4;   remainingMins = 675 - nowMins; }
  else if (nowMins >= 675 && nowMins < 710) { currentPeriod = 5;   remainingMins = 710 - nowMins; }
  else if (nowMins >= 710 && nowMins < 740) { currentPeriod = 6;   remainingMins = 740 - nowMins; }
  else if (nowMins >= 302 && nowMins < 332) { currentPeriod = 100; remainingMins = 332 - nowMins; }

  if (currentPeriod == -1 || lastRecordedPeriod == currentPeriod) return;

  // FIX #4: skip near-zero remaining time
  int remainingSecs = remainingMins * 60;
  if (remainingSecs < 10) {
    Serial.printf("[TIME] Period %d has only %ds left — marking done, skipping.\n",
                  currentPeriod, remainingSecs);
    lastRecordedPeriod = currentPeriod;
    saveState();
    return;
  }

  lastRecordedPeriod = currentPeriod;
  saveState(); // FIX #1: persist now so a crash during recording doesn't re-record

  char fileName[48], dateStr[16];
  strftime(dateStr, sizeof(dateStr), "%Y-%m-%d.wav", &timeinfo);
  snprintf(fileName, sizeof(fileName), "Period_%d_%s", currentPeriod, dateStr);

  enforceSDSpace();
  recordClassAudio(String(fileName), remainingSecs);
  processPendingUploads();
  checkOTA();
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pixel.begin();
  setColor(50, 20, 0); // AMBER = booting

  // FIX #11: hardware watchdog — auto-reboot if stuck
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  // SD
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] Mount failed!");
    setColor(50, 0, 0);
    sdReady = false;
    // FIX #12: sdReady=false prevents any SD calls below if mount failed
  } else {
    sdReady = true;
    SD.mkdir("/pending");
    SD.mkdir("/retries"); // FIX #8
    loadState();          // FIX #1
  }

  initI2S();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet, primaryDNS);
  WiFi.begin(ssid, password);

  Serial.print("[WiFi] Connecting");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    esp_task_wdt_reset();
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected: " + WiFi.localIP().toString());
    syncTime();
  } else {
    // FIX #6: don't stall — keepWiFiAlive() in loop() will handle reconnection
    Serial.println("[WiFi] Boot connect failed. Will retry in loop.");
  }

  checkOTA();
  setColor(0, 0, 0);
}

void loop() {
  esp_task_wdt_reset(); // FIX #11
  keepWiFiAlive();
  checkTimetableAndRecord();
  delay(1000);
}