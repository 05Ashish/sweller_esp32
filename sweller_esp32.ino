#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <driver/i2s.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

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

#define NEOPIXEL_PIN    48
#define NEOPIXEL_COUNT  1

// ============================================================================
// CONFIGURATION
// ============================================================================
#define SAMPLE_RATE         16000
#define BITS_PER_SAMPLE     16
#define I2S_BUFFER_SIZE     4096
#define CHUNK_SIZE          2048        // Fixed W5500 2KB chunk limit
#define MAX_RETRIES         3
#define TIME_ZONE_OFFSET    19800       // IST is UTC + 5:30 (19800 seconds)

byte mac[6];
IPAddress raspberryPi(192, 168, 1, 239); // YOUR PI'S ACTUAL ETHERNET IP
const uint16_t raspberryPiPort = 5000;

// --- ADD THESE 4 LINES ---
IPAddress staticIP(192, 168, 1, 65);     // The forced IP for this ESP32
IPAddress gateway(192, 168, 1, 1);       // Copied from your ipconfig
IPAddress subnet(255, 255, 255, 0);      // Copied from your ipconfig
IPAddress dns(8, 8, 8, 8);               // Google DNS (needed for NTP time sync)

// --- STATUS LED COLORS ---
#define COLOR_BOOT      0x0000FF  // Blue
#define COLOR_RECORDING 0x00FF00  // Green
#define COLOR_READY     0xFFFF00  // Yellow
#define COLOR_UPLOADING 0xFF00FF  // Purple
#define COLOR_ERROR     0xFF0000  // Red
#define COLOR_IDLE      0x000000  // Off

Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
EthernetClient ethClient;
EthernetUDP Udp;
SPIClass sdSPI(HSPI); 

// ============================================================================
// TIMETABLE STRUCTURE (Minutes since Midnight)
// ============================================================================
struct ClassPeriod {
  const char* name;
  int startMin;
  int endMin;
};

// 7:50 AM = (7 * 60) + 50 = 470
ClassPeriod schedule[] = {
  // --- REGULAR SCHEDULE ---
  {"P1", 505, 540},  // 08:25 - 09:00
  {"P2", 540, 575},  // 09:00 - 09:35
  {"P3", 575, 610},  // 09:35 - 10:10
  // LUNCH (10:10 - 10:40) skipped automatically
  {"P4", 640, 675},  // 10:40 - 11:15
  {"P5", 675, 710},  // 11:15 - 11:50
  {"CT", 710, 740},  // 11:50 - 12:20

  // --- 10-MINUTE TEST PERIODS (Ends at 4:00 PM) ---
  {"T1", 740, 750},  // 12:20 - 12:30
  {"T2", 750, 760},  // 12:30 - 12:40
  {"T3", 760, 770},  // 12:40 - 12:50
  {"T4", 770, 780},  // 12:50 - 13:00
  {"T5", 780, 790},  // 13:00 - 13:10
  {"T6", 790, 800},  // 13:10 - 13:20
  {"T7", 800, 810},  // 13:20 - 13:30
  {"T8", 810, 820},  // 13:30 - 13:40
  {"T9", 820, 830},  // 13:40 - 13:50
  {"T10", 830, 840}, // 13:50 - 14:00
  {"T11", 840, 850}, // 14:00 - 14:10 
  {"T12", 850, 860}, // 14:10 - 14:20
  {"T13", 860, 870}, // 14:20 - 14:30
  {"T14", 870, 880}, // 14:30 - 14:40
  {"T15", 935, 965}, // 14:40 - 14:50
  //{"T16", 890, 900}, // 14:50 - 15:00
  //{"T17", 900, 910}, // 15:00 - 15:10
  //{"T18", 910, 920}, // 15:10 - 15:20
  //{"T19", 920, 930}, // 15:20 - 15:30
  //{"T20", 930, 940}, // 15:30 - 15:40
  //{"T21", 940, 950}, // 15:40 - 15:50
  //{"T22", 950, 960}  // 15:50 - 16:00
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
      return epoch + TIME_ZONE_OFFSET; // Return local time (IST)
    }
    delay(10);
  }
  Udp.stop();
  return 0; // Sync failed
}

int getCurrentMinuteOfDay() {
  unsigned long currentEpoch = baseEpochTime + ((millis() - bootMillis) / 1000);
  return (currentEpoch % 86400) / 60; // Returns 0 to 1439
}

long getCurrentSecondOfDay() {
  unsigned long currentEpoch = baseEpochTime + ((millis() - bootMillis) / 1000);
  return (currentEpoch % 86400); // Returns 0 to 86399
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
  pinMode(ETH_CS, OUTPUT); digitalWrite(ETH_CS, HIGH);

// Initialize Ethernet
  WiFi.mode(WIFI_STA); delay(100);
  WiFi.macAddress(mac);
  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS);
  Ethernet.init(ETH_CS);
  
  // --- REPLACE THE DHCP LOOP WITH THIS ---
  Serial.println("Forcing Static IP (Bypassing DHCP)...");
  Ethernet.begin(mac, staticIP, dns, gateway, subnet);
  
  // Give the school switch a few seconds to wake up the port
  delay(5000); 
  
  Serial.print("Ethernet OK. Forced IP: "); 
  Serial.println(Ethernet.localIP());

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

  // Check the timetable to see if we should be recording right now
  for (int i = 0; i < numPeriods; i++) {
    // Stagger logic: 3 minutes (180s) + up to 60 extra seconds based on MAC address
    long macOffset = mac[5] % 60; 
    long uploadStartSec = (schedule[i].endMin * 60) - 180 - macOffset; 
    long classStartSec = schedule[i].startMin * 60;

    if (currentSec >= classStartSec && currentSec < uploadStartSec) {
      Serial.printf("\n--- Class Started: %s ---\n", schedule[i].name);
      
      String filename = "/recordings/ROOM_B_" + String(schedule[i].name) + ".wav";
      
      // Pass the exact SECOND we want it to stop, rather than the minute
      recordAudioFileUntil(filename, uploadStartSec);
      
      // Start upload sequence
      setLED(COLOR_READY); delay(1000);
      setLED(COLOR_UPLOADING);
      
      if (uploadFileToRaspberryPi(filename)) {
        SD.remove(filename);
        Serial.println("✓ Upload successful and file deleted.");
      } else {
        Serial.println("✗ Upload failed. Keeping file on SD.");
      }
      
      // Wait until the period fully ends before scanning for the next one
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

  // 1. Write an empty placeholder header to reserve the first 44 bytes
  WavHeader header;
  memset(&header, 0, sizeof(WavHeader));
  file.write((uint8_t*)&header, sizeof(WavHeader));

  int32_t* i2sBuf = (int32_t*)malloc(I2S_BUFFER_SIZE);
  int16_t* outBuf = (int16_t*)malloc(I2S_BUFFER_SIZE / 2);
  
  // Use a 32-bit unsigned int. This can count up to 4,294,967,295 bytes (4GB).
  uint32_t totalDataBytesWritten = 0; 
  
  setLED(COLOR_RECORDING);
  
  // Calculate Hour, Minute, and Second for the debug print
  int displayHour = stopSecond / 3600;
  int displayMin = (stopSecond % 3600) / 60;
  int displaySec = stopSecond % 60;
  Serial.printf("Recording until %02d:%02d:%02d...\n", displayHour, displayMin, displaySec);

  // 2. Loop and record using SECONDS instead of minutes
  while (getCurrentSecondOfDay() < stopSecond) {
    size_t bytesRead = 0;
    
    // Read from microphone
    esp_err_t result = i2s_read(I2S_PORT, i2sBuf, I2S_BUFFER_SIZE, &bytesRead, portMAX_DELAY);
    
    if (result == ESP_OK && bytesRead > 0) {
      int samples = bytesRead / 4;
      
      // Downsample 32-bit to 16-bit
      for (int i = 0; i < samples; i++) {
        outBuf[i] = (int16_t)(i2sBuf[i] >> 14);
      }
      
      // Write the 16-bit audio to the SD card
      size_t written = file.write((uint8_t*)outBuf, samples * 2);
      
      // 3. Keep a strictly accurate count of every byte written
      if (written > 0) {
        totalDataBytesWritten += written;
      }
    }
    yield();
  }
  
  free(i2sBuf); free(outBuf);

  // 4. Create the final, accurate WAV header using our byte count
  WavHeader h;
  memcpy(h.riff, "RIFF", 4); 
  h.fileSize = totalDataBytesWritten + 36; // File size minus 8 bytes
  memcpy(h.wave, "WAVE", 4); 
  memcpy(h.fmt, "fmt ", 4);
  h.fmtSize = 16; 
  h.audioFormat = 1; 
  h.numChannels = 1;
  h.sampleRate = SAMPLE_RATE; 
  h.bitsPerSample = BITS_PER_SAMPLE;
  h.byteRate = SAMPLE_RATE * 2; 
  h.blockAlign = 2;
  memcpy(h.data, "data", 4); 
  h.dataSize = totalDataBytesWritten; // Exact size of the audio data
  
  // 5. Rewind to byte 0 and overwrite the placeholder with the real header
  file.seek(0); 
  file.write((uint8_t*)&h, sizeof(WavHeader));
  
  // 6. Ensure everything is physically written to the SD card before closing
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
  ethClient.println("Connection: close\r\n");
  
  uint8_t* buffer = (uint8_t*)malloc(CHUNK_SIZE);
  if (!buffer) { ethClient.stop(); file.close(); return false; }
  
  while (file.available()) {
    size_t bytesRead = file.read(buffer, min((size_t)CHUNK_SIZE, (size_t)file.available()));
    ethClient.write(buffer, bytesRead);
    yield();
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