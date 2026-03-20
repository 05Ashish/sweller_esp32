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

// ============================================================================
// HARDWARE & PIN CONFIGURATION
// ============================================================================
const int CURRENT_VERSION = 15;

#define NEO_PIN   48

// SD Card Pins (SPI)
#define SD_CS     10
#define SPI_MOSI  11
#define SPI_SCK   12
#define SPI_MISO  13

// INMP441 Microphone Pins (I2S)
#define I2S_WS    15
#define I2S_SCK   16
#define I2S_SD    17
#define I2S_PORT  I2S_NUM_0

#define SAMPLE_RATE      16000
#define I2S_BUFFER_SIZE  1024

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

String piServerIP       = "http://192.168.1.3:5000";
String githubVersionUrl = "https://raw.githubusercontent.com/05Ashish/sweller_esp32/main/version.txt";
String githubFirmwareUrl= "https://raw.githubusercontent.com/05Ashish/sweller_esp32/main/build/esp32.esp32.esp32s3/sweller_esp32.ino.bin";

const char* ntpServer        = "pool.ntp.org";
const long  gmtOffset_sec    = 19800; // IST: GMT+5:30
const int   daylightOffset_sec = 0;

int lastRecordedPeriod = -1;

// ============================================================================
// HELPER: VISUAL STATUS
// ============================================================================
void setColor(int r, int g, int b) {
  pixel.setPixelColor(0, pixel.Color(g, r, b));
  pixel.show();
}

// ============================================================================
// AUDIO & I2S SETUP
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
    .bck_io_num    = I2S_SCK,
    .ws_io_num     = I2S_WS,
    .data_out_num  = I2S_PIN_NO_CHANGE,
    .data_in_num   = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void writeWavHeader(File& file, uint32_t dataSize) {
  byte header[44];
  uint32_t fileSize = dataSize + 36;
  uint32_t byteRate = SAMPLE_RATE * 2;

  memcpy(header, "RIFF", 4);
  header[4]  = (byte)(fileSize & 0xFF);
  header[5]  = (byte)((fileSize >> 8)  & 0xFF);
  header[6]  = (byte)((fileSize >> 16) & 0xFF);
  header[7]  = (byte)((fileSize >> 24) & 0xFF);
  memcpy(header + 8,  "WAVE", 4);
  memcpy(header + 12, "fmt ", 4);
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  header[20] = 1;  header[21] = 0;  // PCM
  header[22] = 1;  header[23] = 0;  // Mono
  header[24] = (byte)(SAMPLE_RATE & 0xFF);
  header[25] = (byte)((SAMPLE_RATE >> 8)  & 0xFF);
  header[26] = (byte)((SAMPLE_RATE >> 16) & 0xFF);
  header[27] = (byte)((SAMPLE_RATE >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF);
  header[29] = (byte)((byteRate >> 8)  & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = 2;  header[33] = 0;  // Block align
  header[34] = 16; header[35] = 0;  // Bits per sample
  memcpy(header + 36, "data", 4);
  header[40] = (byte)(dataSize & 0xFF);
  header[41] = (byte)((dataSize >> 8)  & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF);
  header[43] = (byte)((dataSize >> 24) & 0xFF);

  file.seek(0);
  file.write(header, 44);
}

void recordClassAudio(String fileName, int durationSeconds) {
  String filePath = "/pending/" + fileName;
  File audioFile  = SD.open(filePath, FILE_WRITE);
  if (!audioFile) return;

  Serial.println("[REC] Recording to: " + filePath +
                 " for " + String(durationSeconds / 60) + " mins");
  setColor(50, 0, 0); // RED

  audioFile.seek(44); // Leave room for WAV header

  size_t   bytesRead;
  int16_t  i2sData[I2S_BUFFER_SIZE / 2];
  uint32_t totalDataBytes  = 0;
  unsigned long startMillis      = millis();
  unsigned long recordDurationMs = (unsigned long)durationSeconds * 1000UL;

  while (millis() - startMillis < recordDurationMs) {
    i2s_read(I2S_PORT, &i2sData, I2S_BUFFER_SIZE, &bytesRead, portMAX_DELAY);
    if (bytesRead > 0) {
      audioFile.write((const byte*)i2sData, bytesRead);
      totalDataBytes += bytesRead;
    }
    yield();
  }

  writeWavHeader(audioFile, totalDataBytes);
  audioFile.close();
  setColor(0, 50, 0); // GREEN
}

// ============================================================================
// NETWORK & NTP
// ============================================================================

// FIX: Unconditionally syncs time. Call this once after WiFi is confirmed up.
void syncTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  Serial.print("[NTP] Waiting for time sync");
  struct tm timeinfo;
  // Block until NTP responds (up to ~10 s)
  unsigned long start = millis();
  while (!getLocalTime(&timeinfo) && millis() - start < 10000) {
    Serial.print(".");
    delay(500);
  }
  if (getLocalTime(&timeinfo)) {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.printf("\n[NTP] Time synced: %s\n", buf);
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
    setColor(0, 0, 0);   delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Reconnected. Re-syncing time...");
    syncTime(); // FIX: re-sync time whenever WiFi comes back up
  }
}

// ============================================================================
// SD CARD HELPERS
// ============================================================================
void enforceSDSpace() {
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes  = SD.usedBytes();
  float freeSpace = 100.0f * ((float)(totalBytes - usedBytes) / (float)totalBytes);

  while (freeSpace < 5.0f) {
    File dir         = SD.open("/pending");
    File oldestFile  = dir.openNextFile();
    if (oldestFile) {
      String path = String("/pending/") + oldestFile.name();
      oldestFile.close();
      SD.remove(path);
    } else {
      break;
    }
    usedBytes  = SD.usedBytes();
    freeSpace  = 100.0f * ((float)(totalBytes - usedBytes) / (float)totalBytes);
  }
}

// ============================================================================
// LOCAL UPLOAD
// ============================================================================
void processPendingUploads() {
  if (WiFi.status() != WL_CONNECTED) return;

  File dir = SD.open("/pending");
  if (!dir) return;

  File uploadFile = dir.openNextFile();

  // Jitter only if there is something to upload
  if (uploadFile) {
    int jitterMillis = random(30000, 120000);
    unsigned long startJitter = millis();
    while (millis() - startJitter < (unsigned long)jitterMillis) {
      keepWiFiAlive();
      yield();
      delay(10);
    }
  }

  while (uploadFile) {
    if (!uploadFile.isDirectory()) {
      String fileName = uploadFile.name();
      String filePath = String("/pending/") + fileName;

      HTTPClient http;
      http.setTimeout(15000);
      http.begin(piServerIP + "/upload");
      http.addHeader("X-Filename", fileName);

      int httpCode = http.sendRequest("POST", &uploadFile, uploadFile.size());
      uploadFile.close();

      if (httpCode == 200) {
        SD.remove(filePath);
      }
      http.end();
    }
    yield();
    uploadFile = dir.openNextFile();
  }
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

  int    httpCode = http.GET();
  String version  = "";

  if (httpCode == 200) {
    version = http.getString();
    version.trim();
  }
  http.end();
  return version;
}

void checkOTA() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("\n[OTA] Checking GitHub for updates...");
  setColor(0, 0, 50); // BLUE

  String serverVersion = getServerVersion();
  if (serverVersion == "") {
    Serial.println("[OTA] Failed to read version from GitHub.");
    setColor(0, 0, 0);
    return;
  }

  int newVersion = serverVersion.toInt();
  Serial.printf("[OTA] Current version: %d | Server version: %d\n",
                CURRENT_VERSION, newVersion);

  if (newVersion > CURRENT_VERSION) {
    Serial.println("[OTA] New firmware available. Downloading...");
    setColor(25, 0, 25); // PURPLE

    WiFiClientSecure client;
    client.setInsecure();
    httpUpdate.rebootOnUpdate(false);

    t_httpUpdate_return ret = httpUpdate.update(client, githubFirmwareUrl);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("[OTA] Update failed: %s\n",
                      httpUpdate.getLastErrorString().c_str());
        setColor(50, 0, 0);
        delay(2000);
        setColor(0, 0, 0);
        break;
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("[OTA] No updates available.");
        setColor(0, 0, 0);
        break;
      case HTTP_UPDATE_OK:
        Serial.println("[OTA] Update successful. Rebooting...");
        setColor(0, 50, 0);
        delay(1500);
        ESP.restart();
        break;
    }
  } else {
    Serial.println("[OTA] Firmware is up to date.");
    setColor(0, 0, 0);
  }
}

// ============================================================================
// TIMETABLE LOGIC
// ============================================================================
void checkTimetableAndRecord() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[TIME] getLocalTime failed — skipping timetable check.");
    return;
  }

  int nowMins        = (timeinfo.tm_hour * 60) + timeinfo.tm_min;
  int currentPeriod  = -1;
  int remainingMins  = 0;

  // Period 1: 8:25–9:00
  if      (nowMins >= 505 && nowMins < 540) { currentPeriod = 1; remainingMins = 540 - nowMins; }
  // Period 2: 9:00–9:35
  else if (nowMins >= 540 && nowMins < 575) { currentPeriod = 2; remainingMins = 575 - nowMins; }
  // Period 3: 9:35–10:10
  else if (nowMins >= 575 && nowMins < 610) { currentPeriod = 3; remainingMins = 610 - nowMins; }
  // Period 4: 10:40–11:15
  else if (nowMins >= 640 && nowMins < 675) { currentPeriod = 4; remainingMins = 675 - nowMins; }
  // Period 5: 11:15–11:50
  else if (nowMins >= 675 && nowMins < 710) { currentPeriod = 5; remainingMins = 710 - nowMins; }
  // CT:       11:50–12:20
  else if (nowMins >= 710 && nowMins < 740) { currentPeriod = 6; remainingMins = 740 - nowMins; }
  // Test:     5:02–5:32
  else if (nowMins >= 332 && nowMins < 360) { currentPeriod = 100; remainingMins = 360 - nowMins; }

  if (currentPeriod == -1 || lastRecordedPeriod == currentPeriod) return;

  lastRecordedPeriod = currentPeriod;

  // Build filename
  char fileName[48];
  char dateStr[16];
  strftime(dateStr, sizeof(dateStr), "%Y-%m-%d.wav", &timeinfo);
  snprintf(fileName, sizeof(fileName), "Period_%d_%s", currentPeriod, dateStr);

  enforceSDSpace();
  recordClassAudio(String(fileName), remainingMins * 60);
  processPendingUploads();
  checkOTA(); // Only runs once per period, after recording
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pixel.begin();

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] Mount failed!");
    setColor(50, 0, 0);
  } else {
    SD.mkdir("/pending");
  }

  initI2S();

  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet, primaryDNS);
  WiFi.begin(ssid, password);

  Serial.print("[WiFi] Connecting");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected: " + WiFi.localIP().toString());
    syncTime(); // FIX: explicit, blocking time sync before anything else
  } else {
    Serial.println("[WiFi] Failed to connect on boot.");
  }

  // Single boot-time OTA check — time is synced so timetable won't double-fire
  checkOTA();
}

void loop() {
  keepWiFiAlive();
  checkTimetableAndRecord();
  delay(1000);
}