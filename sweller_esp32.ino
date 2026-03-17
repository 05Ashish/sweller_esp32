#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <driver/i2s.h>
#include <Ethernet.h>        
#include <EthernetUdp.h>
#include <WiFi.h>            
#include <Update.h>
#include <Adafruit_NeoPixel.h>  // Add this back

// ... keep your existing pin defines ...

#define NEOPIXEL_PIN    48      // The built-in LED on ESP32-S3
#define NEOPIXEL_COUNT  1

Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
// FIRMWARE VERSION
// ============================================================================
const int CURRENT_FIRMWARE_VERSION = 3; 

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

#define ETH_CS          9
#define ETH_SCK         36
#define ETH_MISO        37
#define ETH_MOSI        35

// ============================================================================
// CONFIGURATION
// ============================================================================
#define SAMPLE_RATE         16000
#define BITS_PER_SAMPLE     16
#define I2S_BUFFER_SIZE     4096
#define CHUNK_SIZE          2048
#define MAX_RETRIES         3
#define TIME_ZONE_OFFSET    19800       // IST is UTC + 5:30 

byte mac[6];
IPAddress raspberryPi(192, 168, 1, 239); // The local Edge Server
const uint16_t raspberryPiPort = 5000;

IPAddress staticIP(192, 168, 1, 152);    // Room A's IP
IPAddress gateway(192, 168, 1, 1);       
IPAddress subnet(255, 255, 255, 0);      
IPAddress dns(192, 168, 1, 1);           // Uses router for DNS             

EthernetClient ethClient;
EthernetUDP Udp;
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

unsigned long baseEpochTime = 0;
unsigned long bootMillis = 0;

struct WavHeader {
  char riff[4]; uint32_t fileSize; char wave[4]; char fmt[4];
  uint32_t fmtSize; uint16_t audioFormat; uint16_t numChannels;
  uint32_t sampleRate; uint32_t byteRate; uint16_t blockAlign;
  uint16_t bitsPerSample; char data[4]; uint32_t dataSize;
} __attribute__((packed));

// ============================================================================
// LOCAL OTA UPDATE LOGIC
// ============================================================================
void checkForUpdates() {
  Serial.printf("\nChecking Local Pi for updates (Current Version: %d)...\n", CURRENT_FIRMWARE_VERSION);
  
  if (ethClient.connect(raspberryPi, raspberryPiPort)) {
    ethClient.println("GET /firmware/version.txt HTTP/1.1");
    ethClient.printf("Host: %s\r\n", raspberryPi.toString().c_str());
    ethClient.println("Connection: close\r\n");
    
    // Skip HTTP Headers
    while (ethClient.connected()) {
      String line = ethClient.readStringUntil('\n');
      if (line == "\r") break; 
    }
    
    String versionStr = ethClient.readStringUntil('\n');
    versionStr.trim();
    int availableVersion = versionStr.toInt();
    ethClient.stop();
    
    if (availableVersion > CURRENT_FIRMWARE_VERSION) {
      Serial.printf("New version %d found on Pi! Downloading...\n", availableVersion);
      
      if (ethClient.connect(raspberryPi, raspberryPiPort)) {
        ethClient.println("GET /firmware/firmware.bin HTTP/1.1");
        ethClient.printf("Host: %s\r\n", raspberryPi.toString().c_str());
        ethClient.println("Connection: close\r\n");
        
        long contentLength = 0;
        while (ethClient.connected()) {
          String line = ethClient.readStringUntil('\n');
          line.trim();
          if (line.startsWith("Content-Length: ")) {
            contentLength = line.substring(16).toInt();
          }
          if (line.length() == 0) break; // End of headers
        }
        
        if (contentLength > 0 && Update.begin(contentLength)) {
          size_t written = 0;
          uint8_t buffer[1024]; // 1KB chunks
          
          Serial.print("Progress: ");

          while (ethClient.connected() && written < contentLength) {
            size_t available = ethClient.available();
            if (available > 0) {
              size_t bytesToRead = min(available, sizeof(buffer));
              int c = ethClient.read(buffer, bytesToRead);
              Update.write(buffer, c);
              written += c;

              // Print a dot every ~50KB to show it's alive
              if (written % (1024 * 50) < sizeof(buffer)) {
                Serial.print(".");
              }
            }
            // THIS is the magic watchdog feeder. 
            // delay(1) forces the OS to breathe, yield() does not.
            delay(1); 
          }
          Serial.println(); // Print newline after progress dots

          if (Update.end() && Update.isFinished()) {
            Serial.println("OTA Update successfully completed! Rebooting...");
            delay(1000);
            ESP.restart(); 
          } else {
            Serial.printf("OTA Write Error #: %d\n", Update.getError());
          }
        }
        ethClient.stop();
      }
    } else {
      Serial.println("Firmware is up to date.");
    }
  } else {
    Serial.println("Failed to connect to Pi for update check.");
  }
}

// ============================================================================
// NTP TIME SYNC
// ============================================================================
unsigned long getNTPTime() {
  pixel.setPixelColor(0, pixel.Color(20, 20, 20)); // Dim White
  pixel.show();
  const char timeServer[] = "216.239.35.0"; // Bypasses DNS entirely
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
    delay(100);
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
// SETUP
// ============================================================================
void setup() {
  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(50, 20, 0)); // Dim Orange (Warm startup)
  pixel.show();
  Serial.begin(115200);
  delay(3000); 
  Serial.println("\n--- SMART TIMETABLE SYSTEM STARTING ---");
  
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
  pinMode(ETH_CS, OUTPUT); digitalWrite(ETH_CS, HIGH);

  WiFi.mode(WIFI_STA); delay(100);
  WiFi.macAddress(mac);
  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS);
  Ethernet.init(ETH_CS);
  
  Serial.println("Forcing Static IP (Bypassing DHCP)...");
  Ethernet.begin(mac, staticIP, dns, gateway, subnet);
  delay(5000); 
  
  Serial.print("Ethernet OK. Forced IP: "); 
  Serial.println(Ethernet.localIP());

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

  // --- CHECK THE PI FOR UPDATES EVERY BOOT ---
  checkForUpdates();

  delay(2000);

  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI, 10000000)) {
    Serial.println("SD PERMANENT FAILURE");
    while(1) delay(1000);
  }
  if (!SD.exists("/recordings")) SD.mkdir("/recordings");

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
  pixel.setPixelColor(0, pixel.Color(0, 50, 0)); // Solid Green (System is safe)
  pixel.show();
}

// ============================================================================
// MAIN TIMETABLE LOOP
// ============================================================================
void loop() {
  int currentMin = getCurrentMinuteOfDay();
  long currentSec = getCurrentSecondOfDay();
  
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
      String filename = "/recordings/ROOM_A_" + String(schedule[i].name) + ".wav";
      
      recordAudioFileUntil(filename, uploadStartSec);
      delay(1000);
      
      if (uploadFileToRaspberryPi(filename)) {
        SD.remove(filename);
        Serial.println("✓ Upload successful and file deleted.");
      } else {
        Serial.println("✗ Upload failed. Keeping file on SD.");
        delay(3000);
      }
      
      while(getCurrentMinuteOfDay() < schedule[i].endMin) {
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
  
  int retries = 0;
  while (retries < MAX_RETRIES) {
    if (ethClient.connect(raspberryPi, raspberryPiPort)) break;
    retries++;
    delay(2000);
    if (retries >= MAX_RETRIES) { file.close(); return false; }
  }
  
  ethClient.println("POST /upload HTTP/1.1");
  ethClient.printf("Host: %s\r\n", raspberryPi.toString().c_str());
  ethClient.println("Content-Type: audio/wav");
  ethClient.printf("Content-Length: %u\r\n", fileSize);
  ethClient.printf("X-Filename: %s\r\n", baseName.c_str());
  ethClient.println("X-MD5: SKIP"); 
  ethClient.printf("X-File-Size: %u\r\n", fileSize);
  ethClient.println("Connection: close\r\n\r\n"); 
  
  uint8_t* buffer = (uint8_t*)malloc(CHUNK_SIZE);
  if (!buffer) { ethClient.stop(); file.close(); return false; }
  
  unsigned long uploadStart = millis();
  
  while (file.available()) {
    if (millis() - uploadStart > 300000) { 
      Serial.println("✗ NETWORK TIMEOUT: Upload took longer than 5 minutes.");
      ethClient.stop(); free(buffer); file.close(); return false; 
    }
    
    size_t bytesRead = file.read(buffer, min((size_t)CHUNK_SIZE, (size_t)file.available()));
    ethClient.write(buffer, bytesRead);
    delay(1); // Force watchdog feed during upload too!
  }
  
  ethClient.flush(); free(buffer); file.close();
  
  unsigned long timeout = millis();
  while (!ethClient.available()) {
    if (millis() - timeout > 60000 || !ethClient.connected()) { ethClient.stop(); return false; }
    delay(100);
  }
  
  String response = "";
  timeout = millis();
  while (ethClient.available() || (ethClient.connected() && millis() - timeout < 5000)) {
    if (ethClient.available()) { response += (char)ethClient.read(); timeout = millis(); }
    delay(1);
  }
  ethClient.stop();
  
  return (response.indexOf("200 OK") > 0 || response.indexOf("\"status\": \"success\"") > 0);
}