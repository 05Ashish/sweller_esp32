// ========================= ESP32 CODE (Arduino IDE) =========================
// ESP32 Classroom Audio Recorder + Auto Uploader (RAW)
// - No buttons, no OLED
// - On power: record fixed duration -> upload to laptop
// - Upload method: RAW HTTP body (most reliable)
// - After success: rename to .sent.wav

#include <Arduino.h>
#include <driver/i2s.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>

// ------------------- YOUR DETAILS -------------------
#define DEVICE_ID        "CLASSROOM_01"
#define WIFI_SSID        "Airtel_NEGI"
#define WIFI_PASS        "mnian281996"

#define PI_HOST          "192.168.1.7"
#define PI_PORT          8000

// ------------------- RECORDING SETTINGS -------------------
#define SAMPLE_RATE           16000

// 3 minutes (testing)
#define RECORD_TIME_MS        180000UL

// 30 minutes (final) -> use this instead when ready
// #define RECORD_TIME_MS     1800000UL

// ------------------- SD SETTINGS -------------------
#define SD_SPI_FREQ           10000000

// ------------------- PINS (ESP32 WROVER typical) -------------------
// INMP441 I2S
#define I2S_WS   25
#define I2S_SCK  26
#define I2S_SD   32
#define I2S_PORT I2S_NUM_0

// SD Card SPI
#define SD_CS    5
#define SD_SCK   18
#define SD_MISO  19
#define SD_MOSI  23

SPIClass sdSPI(VSPI);

// =========================================================
// WAV HEADER (44 bytes) - safe, no struct padding issues
// =========================================================
void writeWavHeader44(File &file, uint32_t sampleRate, uint32_t dataSize) {
  uint32_t chunkSize = 36 + dataSize;
  uint32_t byteRate  = sampleRate * 1 * 16 / 8;
  uint16_t blockAlign = 1 * 16 / 8;

  uint8_t h[44];

  // RIFF
  h[0] = 'R'; h[1] = 'I'; h[2] = 'F'; h[3] = 'F';
  h[4] = (chunkSize) & 0xFF;
  h[5] = (chunkSize >> 8) & 0xFF;
  h[6] = (chunkSize >> 16) & 0xFF;
  h[7] = (chunkSize >> 24) & 0xFF;

  // WAVE
  h[8]  = 'W'; h[9]  = 'A'; h[10] = 'V'; h[11] = 'E';

  // fmt
  h[12] = 'f'; h[13] = 'm'; h[14] = 't'; h[15] = ' ';
  h[16] = 16; h[17] = 0; h[18] = 0; h[19] = 0;   // PCM fmt chunk size
  h[20] = 1;  h[21] = 0;                          // AudioFormat = 1 (PCM)
  h[22] = 1;  h[23] = 0;                          // NumChannels = 1

  // SampleRate
  h[24] = (sampleRate) & 0xFF;
  h[25] = (sampleRate >> 8) & 0xFF;
  h[26] = (sampleRate >> 16) & 0xFF;
  h[27] = (sampleRate >> 24) & 0xFF;

  // ByteRate
  h[28] = (byteRate) & 0xFF;
  h[29] = (byteRate >> 8) & 0xFF;
  h[30] = (byteRate >> 16) & 0xFF;
  h[31] = (byteRate >> 24) & 0xFF;

  // BlockAlign
  h[32] = (blockAlign) & 0xFF;
  h[33] = (blockAlign >> 8) & 0xFF;

  // BitsPerSample
  h[34] = 16; h[35] = 0;

  // data
  h[36] = 'd'; h[37] = 'a'; h[38] = 't'; h[39] = 'a';
  h[40] = (dataSize) & 0xFF;
  h[41] = (dataSize >> 8) & 0xFF;
  h[42] = (dataSize >> 16) & 0xFF;
  h[43] = (dataSize >> 24) & 0xFF;

  file.seek(0);
  file.write(h, 44);
}

// =========================================================
// I2S CONFIG (INMP441)
// =========================================================
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 8,
  .dma_buf_len = 256,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = -1,
  .data_in_num = I2S_SD
};

// =========================================================
// HELPERS
// =========================================================
void wifiOff() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(200);
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("WiFi connecting to: %s\n", WIFI_SSID);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("WiFi FAILED");
  return false;
}

String getNextFilename() {
  // /rec/CLASSROOM_01_0001.wav
  const char* folder = "/rec";
  if (!SD.exists(folder)) SD.mkdir(folder);

  for (int i = 1; i < 10000; i++) {
    char name[64];
    snprintf(name, sizeof(name), "/rec/%s_%04d.wav", DEVICE_ID, i);

    String sentName = String(name);
    sentName.replace(".wav", ".sent.wav");

    if (!SD.exists(name) && !SD.exists(sentName)) {
      return String(name);
    }
  }
  return "/rec/FAIL.wav";
}

bool markAsSent(const String &path) {
  String sentPath = path;
  sentPath.replace(".wav", ".sent.wav");

  if (SD.rename(path, sentPath)) {
    Serial.printf("Marked sent: %s\n", sentPath.c_str());
    return true;
  }

  Serial.println("Mark sent failed");
  return false;
}

// =========================================================
// RECORDING
// =========================================================
String recordFixedWav() {
  String filename = getNextFilename();
  Serial.printf("\nRecording -> %s\n", filename.c_str());

  // Ensure it doesn't exist (safety)
  if (SD.exists(filename)) SD.remove(filename);

  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("ERROR: SD open failed");
    return "";
  }

  // Write header placeholder (dataSize = 0)
  writeWavHeader44(file, SAMPLE_RATE, 0);

  // Force audio to start at byte 44
  file.seek(44);

  // Init I2S
  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
    Serial.println("ERROR: i2s_driver_install failed");
    file.close();
    return "";
  }

  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
    Serial.println("ERROR: i2s_set_pin failed");
    i2s_driver_uninstall(I2S_PORT);
    file.close();
    return "";
  }

  i2s_set_sample_rates(I2S_PORT, SAMPLE_RATE);
  i2s_zero_dma_buffer(I2S_PORT);

  const int samples_to_read = 1024;
  const int i2s_buffer_size = samples_to_read * 4;

  int32_t *i2s_buff = (int32_t*) calloc(samples_to_read, sizeof(int32_t));
  int16_t *out_buff = (int16_t*) calloc(samples_to_read, sizeof(int16_t));

  if (!i2s_buff || !out_buff) {
    Serial.println("ERROR: malloc failed");
    if (i2s_buff) free(i2s_buff);
    if (out_buff) free(out_buff);
    i2s_driver_uninstall(I2S_PORT);
    file.close();
    return "";
  }

  unsigned long start = millis();
  uint32_t totalAudioBytes = 0;

  while (millis() - start < RECORD_TIME_MS) {
    size_t bytes_read = 0;
    esp_err_t result = i2s_read(I2S_PORT, i2s_buff, i2s_buffer_size, &bytes_read, portMAX_DELAY);
    if (result != ESP_OK || bytes_read == 0) continue;

    int samples_read = bytes_read / 4;

    // Convert 32-bit -> 16-bit PCM
    for (int i = 0; i < samples_read; i++) {
      out_buff[i] = (int16_t)(i2s_buff[i] >> 14);
    }

    size_t bytes_to_write = samples_read * 2;
    size_t bytes_written = file.write((uint8_t*)out_buff, bytes_to_write);

    if (bytes_written != bytes_to_write) {
      Serial.printf("SD write error: %d/%d\n", (int)bytes_written, (int)bytes_to_write);
      break;
    }

    totalAudioBytes += bytes_written;

    static uint32_t lastFlush = 0;
    if (millis() - lastFlush > 2000) {
      file.flush();
      lastFlush = millis();
    }
  }

  // Cleanup
  i2s_zero_dma_buffer(I2S_PORT);
  i2s_driver_uninstall(I2S_PORT);

  free(i2s_buff);
  free(out_buff);

  // Finalize header with real data size
  file.flush();
  writeWavHeader44(file, SAMPLE_RATE, totalAudioBytes);
  file.flush();
  file.close();

  Serial.printf("Recording done. Audio bytes: %u  File size: %u\n",
                totalAudioBytes, (unsigned)(totalAudioBytes + 44));

  return filename;
}

// =========================================================
// UPLOAD (RAW HTTP BODY)
// =========================================================
bool uploadFileToLaptop_RAW(const String &path) {
  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.println("Upload: cannot open file");
    return false;
  }

  // Sanity: first 4 bytes should be RIFF
  uint8_t hdr[4];
  f.read(hdr, 4);
  f.seek(0);

  if (!(hdr[0] == 'R' && hdr[1] == 'I' && hdr[2] == 'F' && hdr[3] == 'F')) {
    Serial.println("Upload: file is NOT WAV (missing RIFF). Not uploading.");
    f.close();
    return false;
  }

  WiFiClient client;
  if (!client.connect(PI_HOST, PI_PORT)) {
    Serial.println("Upload: cannot connect to laptop");
    f.close();
    return false;
  }

  String filename = path.substring(path.lastIndexOf("/") + 1);

  client.printf(
    "POST /upload_raw?device_id=%s&filename=%s HTTP/1.1\r\n",
    DEVICE_ID,
    filename.c_str()
  );
  client.printf("Host: %s\r\n", PI_HOST);
  client.print("Content-Type: application/octet-stream\r\n");
  client.printf("Content-Length: %u\r\n", (unsigned)f.size());
  client.print("Connection: close\r\n\r\n");

  uint8_t buf[2048];
  while (f.available()) {
    int n = f.read(buf, sizeof(buf));
    if (n > 0) client.write(buf, n);
  }

  f.close();

  unsigned long t0 = millis();
  while (!client.available() && millis() - t0 < 15000) delay(10);

  String resp;
  while (client.available()) resp += (char)client.read();

  Serial.println("Upload response:");
  Serial.println(resp);

  return resp.indexOf("200 OK") != -1;
}

// =========================================================
// SETUP / LOOP
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\nBooting...");

  // SD init
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI, SD_SPI_FREQ)) {
    Serial.println("SD init FAILED");
    while (1) delay(1000);
  }

  Serial.println("SD OK");

  wifiOff();
}

void loop() {
  // 1) Record fixed duration
  String wavPath = recordFixedWav();
  if (wavPath == "") {
    Serial.println("Record failed, retrying in 5s");
    delay(5000);
    return;
  }

  // 2) Upload
  if (connectWiFi()) {
    bool ok = uploadFileToLaptop_RAW(wavPath);
    if (ok) {
      markAsSent(wavPath);
    } else {
      Serial.println("Upload failed. Keeping file for retry later.");
    }
  } else {
    Serial.println("WiFi unavailable. Keeping file for retry later.");
  }

  // 3) Wi-Fi off and repeat
  wifiOff();
  delay(2000);
}


