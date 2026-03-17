#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <driver/i2s.h>
#include <ETH.h>                 // --- NEW: Native ESP32 Ethernet ---
#include <NetworkUDP.h>          // --- NEW: Native UDP ---
#include <NetworkClient.h>       // --- NEW: Unencrypted Client (For the Pi) ---
#include <NetworkClientSecure.h> // --- NEW: Encrypted Client (For GitHub) ---
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================
// GITHUB OTA CONFIGURATION
// ============================================================================
const int CURRENT_FIRMWARE_VERSION = 1; 

// Replace YOUR_USERNAME and YOUR_REPO. 
// MUST be the "raw.githubusercontent.com" links!
const char* versionURL = "https://raw.githubusercontent.com/05Ashish/sweller_esp32/main/version.txt";
const char* binURL = "https://raw.githubusercontent.com/05Ashish/sweller_esp32/main/build/esp32.esp32.esp32s3/sweller_esp32.ino.bin";

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define I2S_WS          4
#define I2S_SCK         5
#define I2S_SD          6
#define I2S_PORT        I2S_NUM_0

#define SD_CS           10
#define SD_SCK          12
#define SD_MISO         13
#define SD_MOSI         11

// W5500 SPI Pins
#define ETH_CS          9
#define ETH_SCK         36
#define ETH_MISO        37
#define ETH_MOSI        35

#define NEOPIXEL_PIN    48
#define NEOPIXEL_COUNT  1

// ============================================================================
// CONFIGURATION
// ============================================================================
#define SAMPLE_RATE         16000
#define BITS_PER_SAMPLE     16
#define I2S_BUFFER_SIZE     4096
#define CHUNK_SIZE          2048
#define MAX_RETRIES         3
#define TIME_ZONE_OFFSET    19800       // IST is UTC + 5:30 

uint8_t mac[6]; // Used for stagger math
IPAddress raspberryPi(192, 168, 1, 239);
const uint16_t raspberryPiPort = 5000;

IPAddress staticIP(192, 168, 1, 65);     // Change this to .152 for Room A!
IPAddress gateway(192, 168, 1, 1);       
IPAddress subnet(255, 255, 255, 0);      
IPAddress dns(8, 8, 8, 8);               

// --- STATUS LED COLORS ---
#define COLOR_BOOT      0x0000FF  // Blue
#define COLOR_RECORDING 0x00FF00  // Green
#define COLOR_READY     0xFFFF00  // Yellow
#define COLOR_UPLOADING 0xFF00FF  // Purple
#define COLOR_ERROR     0xFF0000  // Red
#define COLOR_IDLE      0x000000  // Off
#define COLOR_UPDATE    0x00FFFF  // Cyan (For OTA updates)

Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
NetworkUDP Udp;
SPIClass sdSPI(HSPI); 

// ============================================================================
// TIMETABLE STRUCTURE 
// ============================================================================
struct ClassPeriod {
  const char* name;
  int startMin;
  int endMin;
};

ClassPeriod schedule[] = {
  {"P1", 505, 540},  {"P2", 540, 575},  {"P3", 575, 610},  
  {"P4", 640, 675},  {"P5", 675, 710},  {"CT", 710, 740},  
  {"T1", 740, 750},  {"T2", 750, 760},  {"T3", 760, 770},  
  {"T4", 770, 780},  {"T5", 780, 790},  {"T6", 790, 800},  
  {"T7", 800, 810},  {"T8", 810, 820},  {"T9", 820, 830},  
  {"T10", 830, 840}, {"T11", 840, 850}, {"T12", 850, 860}, 
  {"T13", 860, 870}, {"T14", 870, 880}, {"T15", 935, 965}
};

const int numPeriods = 21;

// Global Time Variables
unsigned long baseEpochTime = 0;
unsigned long bootMillis = 0;

struct WavHeader {
  char riff[4]; uint32_t fileSize; char wave[4]; char fmt[4];
  uint32_t fmtSize; uint16_t audioFormat; uint16_t numChannels;
  uint32_t sampleRate; uint32_t byteRate; uint16_t blockAlign;
  uint16_t bitsPerSample; char data[4]; uint32_t dataSize;
} __attribute__((packed));

void setLED(uint32_t c) { pixel.setPixelColor(0, c); pixel.show(); }

// ============================================================================
// NTP TIME SYNC
// ============================================================================
unsigned long getNTPTime() {
  const char timeServer[] = "pool.ntp.org";
  byte packetBuffer[48];
  memset(packetBuffer, 0, 48);
  packetBuffer[0] = 0b11100011;   
  packetBuffer[1] = 0; packetBuffer[2] = 6; packetBuffer[3] = 0xEC;
  packetBuffer[12] = 49; packetBuffer[13] = 0x4E; packetBuffer[14] = 49; packetBuffer[15] = 52;

  Udp.begin(8888);
  Udp.beginPacket(timeServer, 123);
  Udp.write(packetBuffer, 48);
  Udp.endPacket();

  unsigned long startWait = millis();
  while (millis() - startWait < 3000) {
    if (Udp.parsePacket()) {
      Udp.read(packetBuffer, 48);
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      const unsigned long seventyYears = 2208988800UL;
      unsigned long epoch = secsSince1900 - seventyYears;
      Udp.stop();
      return epoch + TIME_ZONE_OFFSET; 
    }
    delay(10);
  }
  Udp.stop();
  return 0; 
}

int getCurrentMinuteOfDay() {
  unsigned long currentEpoch = baseEpochTime + ((millis() - bootMillis) / 1000);
  return (currentEpoch % 86400) / 60; 
}

long getCurrentSecondOfDay() {
  unsigned long currentEpoch = baseEpochTime + ((millis() - bootMillis) / 1000);
  return (currentEpoch % 86400); 
}

// ============================================================================
// GITHUB OTA CHECK
// ============================================================================
void checkForUpdates() {
  Serial.printf("\nChecking GitHub for updates (Current Version: %d)...\n", CURRENT_FIRMWARE_VERSION);
  
  // 1. Declare the Secure Network Client
  NetworkClientSecure secureClient;
  secureClient.setInsecure(); // Bypass GitHub's Strict Root CA Check

  HTTPClient http;
  http.begin(secureClient, versionURL);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    int newVersion = payload.toInt();

    if (newVersion > CURRENT_FIRMWARE_VERSION) {
      Serial.printf("New version %d found! Starting OTA Download...\n", newVersion);
      setLED(COLOR_UPDATE); // Cyan for OTA
      
      // 2. Pass the secure client into the HTTP updater
      t_httpUpdate_return ret = httpUpdate.update(secureClient, binURL);

      if (ret == HTTP_UPDATE_FAILED) {
        Serial.printf("OTA Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
        setLED(COLOR_ERROR);
        delay(2000);
      }
    } else {
      Serial.println("Firmware is fully up to date.");
    }
  } else {
    Serial.printf("GitHub Version Check Failed. HTTP Code: %d\n", httpCode);
  }
  http.end();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(3000); 
  Serial.println("\n--- SMART TIMETABLE SYSTEM STARTING ---");
  
  pixel.begin(); setLED(COLOR_BOOT);

  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);

  // Initialize Native ETH.h for W5500
  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS);
  if (!ETH.begin(ETH_PHY_W5500, 1, ETH_CS, -1, -1, SPI)) {
    Serial.println("ETH.begin failed");
  }
  
  ETH.config(staticIP, gateway, subnet, dns);
  delay(5000); // Give the switch time to wake the port
  
  Serial.print("Ethernet OK. Forced IP: "); 
  Serial.println(ETH.localIP());
  
  // Grab the MAC address for stagger logic
  ETH.macAddress(mac);

  // Sync Global Time
  Serial.println("Syncing Time with NTP...");
  while (baseEpochTime == 0) {
    baseEpochTime = getNTPTime();
    if (baseEpochTime == 0) {
      Serial.println("NTP Sync failed. Retrying...");
      delay(2000);
    }
  }
  bootMillis = millis();
  
  int currentMin = getCurrentMinuteOfDay();
  Serial.printf("Time Synced! Current Local Time: %02d:%02d\n", currentMin / 60, currentMin % 60);

  // --- CHECK GITHUB FOR UPDATES EVERY BOOT ---
  checkForUpdates();

  // Initialize SD Card
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI, 10000000)) {
    Serial.println("SD PERMANENT FAILURE");
    setLED(COLOR_ERROR);
    while(1) delay(1000);
  }
  if (!SD.exists("/recordings")) SD.mkdir("/recordings");

  // Initialize I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 16,
    .dma_buf_len = 1024
  };
  i2s_pin_config_t pin_config = { .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS, .data_out_num = -1, .data_in_num = I2S_SD };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  
  Serial.println("System Ready. Waiting for next class...");
  setLED(COLOR_IDLE);
}

// ============================================================================
// MAIN TIMETABLE LOOP
// ============================================================================
void loop() {
  int currentMin = getCurrentMinuteOfDay();
  long currentSec = getCurrentSecondOfDay();
  
  // Daily Time Resync (at 2:00 AM) to prevent millisecond drift
  if (currentMin == 120 && (millis() - bootMillis > 86400000)) {
     baseEpochTime = getNTPTime();
     bootMillis = millis();
  }

  for (int i = 0; i < numPeriods; i++) {
    long macOffset = mac[5] % 60; 
    long uploadStartSec = (schedule[i].endMin * 60) - 180 - macOffset; 
    long classStartSec = schedule[i].startMin * 60;

    if (currentSec >= classStartSec && currentSec < uploadStartSec) {
      Serial.printf("\n--- Class Started: %s ---\n", schedule[i].name);
      
      String filename = "/recordings/ROOM_B_" + String(schedule[i].name) + ".wav";
      
      recordAudioFileUntil(filename, uploadStartSec);
      
      setLED(COLOR_READY); delay(1000);
      setLED(COLOR_UPLOADING);
      
      if (uploadFileToRaspberryPi(filename)) {
        SD.remove(filename);
        Serial.println("✓ Upload successful and file deleted.");
      } else {
        Serial.println("✗ Upload failed. Keeping file on SD.");
        setLED(COLOR_ERROR); 
        delay(3000);
      }
      
      while(getCurrentMinuteOfDay() < schedule[i].endMin) {
        setLED(COLOR_IDLE);
        delay(10000); 
      }
    }
  }
  delay(5000); 
}

// ============================================================================
// CORE LOGIC FUNCTIONS
// ============================================================================
void recordAudioFileUntil(String filename, long stopSecond) {
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("✗ Failed to open file for recording.");
    return;
  }

  WavHeader header;
  memset(&header, 0, sizeof(WavHeader));
  file.write((uint8_t*)&header, sizeof(WavHeader));

  int32_t* i2sBuf = (int32_t*)malloc(I2S_BUFFER_SIZE);
  int16_t* outBuf = (int16_t*)malloc(I2S_BUFFER_SIZE / 2);
  
  uint32_t totalDataBytesWritten = 0; 
  setLED(COLOR_RECORDING);
  
  int displayHour = stopSecond / 3600;
  int displayMin = (stopSecond % 3600) / 60;
  int displaySec = stopSecond % 60;
  Serial.printf("Recording until %02d:%02d:%02d...\n", displayHour, displayMin, displaySec);

  while (getCurrentSecondOfDay() < stopSecond) {
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(I2S_PORT, i2sBuf, I2S_BUFFER_SIZE, &bytesRead, portMAX_DELAY);
    
    if (result == ESP_OK && bytesRead > 0) {
      int samples = bytesRead / 4;
      for (int i = 0; i < samples; i++) {
        outBuf[i] = (int16_t)(i2sBuf[i] >> 14);
      }
      size_t written = file.write((uint8_t*)outBuf, samples * 2);
      if (written > 0) {
        totalDataBytesWritten += written;
      }
    }
    yield();
  }
  
  free(i2sBuf); free(outBuf);

  WavHeader h;
  memcpy(h.riff, "RIFF", 4); 
  h.fileSize = totalDataBytesWritten + 36; 
  memcpy(h.wave, "WAVE", 4); 
  memcpy(h.fmt, "fmt ", 4);
  h.fmtSize = 16; h.audioFormat = 1; h.numChannels = 1;
  h.sampleRate = SAMPLE_RATE; h.bitsPerSample = BITS_PER_SAMPLE;
  h.byteRate = SAMPLE_RATE * 2; h.blockAlign = 2;
  memcpy(h.data, "data", 4); h.dataSize = totalDataBytesWritten; 
  
  file.seek(0); 
  file.write((uint8_t*)&h, sizeof(WavHeader));
  file.flush();
  file.close();
  
  Serial.printf("✓ Recording Finished. Saved: %u MB\n", totalDataBytesWritten / (1024 * 1024));
}

bool uploadFileToRaspberryPi(String filename) {
  Serial.printf("\n--- Uploading: %s ---\n", filename.c_str());
  File file = SD.open(filename, FILE_READ);
  if (!file) return false;
  
  size_t fileSize = file.size();
  String baseName = filename.substring(filename.lastIndexOf('/') + 1);
  
  // Create a fast, unencrypted client for the local Raspberry Pi
  NetworkClient localClient; 
  
  int retries = 0;
  while (retries < MAX_RETRIES) {
    if (localClient.connect(raspberryPi, raspberryPiPort)) break;
    retries++;
    delay(2000);
    if (retries >= MAX_RETRIES) { file.close(); return false; }
  }
  
  localClient.println("POST /upload HTTP/1.1");
  localClient.printf("Host: %s\r\n", raspberryPi.toString().c_str());
  localClient.println("Content-Type: audio/wav");
  localClient.printf("Content-Length: %u\r\n", fileSize);
  localClient.printf("X-Filename: %s\r\n", baseName.c_str());
  localClient.println("X-MD5: SKIP"); 
  localClient.printf("X-File-Size: %u\r\n", fileSize);
  localClient.println("Connection: close\r\n\r\n"); 
  
  uint8_t* buffer = (uint8_t*)malloc(CHUNK_SIZE);
  if (!buffer) { localClient.stop(); file.close(); return false; }
  
  unsigned long uploadStart = millis();
  
  while (file.available()) {
    if (millis() - uploadStart > 300000) { 
      Serial.println("✗ NETWORK TIMEOUT: Upload took longer than 5 minutes.");
      localClient.stop(); free(buffer); file.close(); return false; 
    }
    
    size_t bytesRead = file.read(buffer, min((size_t)CHUNK_SIZE, (size_t)file.available()));
    localClient.write(buffer, bytesRead);
    yield();
  }
  
  localClient.flush(); free(buffer); file.close();
  
  unsigned long timeout = millis();
  while (!localClient.available()) {
    if (millis() - timeout > 60000 || !localClient.connected()) { localClient.stop(); return false; }
    delay(100);
  }
  
  String response = "";
  timeout = millis();
  while (localClient.available() || (localClient.connected() && millis() - timeout < 5000)) {
    if (localClient.available()) { response += (char)localClient.read(); timeout = millis(); }
    delay(1);
  }
  localClient.stop();
  
  return (response.indexOf("200 OK") > 0 || response.indexOf("\"status\": \"success\"") > 0);
}