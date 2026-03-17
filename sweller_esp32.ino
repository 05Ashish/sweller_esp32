#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Update.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
// Set this to 0 to FORCE it to download whatever is on the Pi, 
// or set it to your current version to test the "skip" logic.
const int CURRENT_VERSION = 13; 

#define ETH_CS    9
#define ETH_SCK   36
#define ETH_MISO  37
#define ETH_MOSI  35
#define NEO_PIN   48

Adafruit_NeoPixel pixel(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress staticIP(192, 168, 1, 152);
IPAddress dns(192, 168, 1, 1);           // <-- This is the missing line!
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress raspberryPi(192, 168, 1, 239);
const int piPort = 5000;

EthernetClient client;

// ============================================================================
// HELPER: SET COLOR
// ============================================================================
void setColor(int r, int g, int b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// ============================================================================
// BARE-METAL OTA ROUTINE
// ============================================================================
void runOTA() {
  setColor(0, 0, 50); // BLUE: Checking version
  Serial.println("\n[OTA] Connecting to Pi to check version...");

  client.setTimeout(3000);
  if (!client.connect(raspberryPi, piPort)) {
    Serial.println("[OTA] FAILED: Could not connect to Pi.");
    setColor(50, 0, 0); // RED
    return;
  }

  // 1. Get Version
  client.println("GET /firmware/version.txt HTTP/1.1");
  client.println("Host: 192.168.1.239");
  client.println("Connection: close\r\n");

  unsigned long timeout = millis();
  while (client.connected() && !client.available()) {
    delay(1);
    if (millis() - timeout > 3000) { client.stop(); setColor(50, 0, 0); return; }
  }

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break; // Skip headers
  }
  
  int availableVersion = client.readStringUntil('\n').toInt();
  client.stop();
  Serial.printf("[OTA] Pi has Version: %d. We have: %d\n", availableVersion, CURRENT_VERSION);

  // 2. Evaluate
  if (availableVersion <= CURRENT_VERSION) {
    Serial.println("[OTA] Up to date! Skipping download.");
    setColor(250, 250, 130); // GREEN
    return;
  }

  // 3. Download Binary
  setColor(50, 0, 50); // PURPLE: Downloading
  Serial.println("[OTA] Downloading new firmware...");

  if (!client.connect(raspberryPi, piPort)) {
    setColor(50, 0, 0); // RED
    return;
  }

  client.println("GET /firmware/firmware.bin HTTP/1.1");
  client.println("Host: 192.168.1.239");
  client.println("Connection: close\r\n");

  long contentLength = 0;
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.startsWith("Content-Length: ")) {
      contentLength = line.substring(16).toInt();
    }
    if (line.length() == 0) break; 
  }

  if (contentLength > 0 && Update.begin(contentLength)) {
    size_t written = 0;
    uint8_t buffer[1024];

    while (client.connected() && written < contentLength) {
      size_t available = client.available();
      if (available > 0) {
        int c = client.read(buffer, min(available, sizeof(buffer)));
        Update.write(buffer, c);
        written += c;
      }
      //esp_task_wdt_reset(); // KEEP THE WATCHDOG HAPPY
      delay(1); 
    }

    if (Update.end() && Update.isFinished()) {
      Serial.println("[OTA] SUCCESS! Rebooting in 2 seconds...");
      setColor(250, 250, 130); // GREEN
      delay(2000);
      ESP.restart();
    } else {
      Serial.printf("[OTA] ERROR: Update failed. Code: %d\n", Update.getError());
      setColor(50, 0, 0); // RED
    }
  } else {
    Serial.println("[OTA] ERROR: Not enough space or bad content length.");
    setColor(50, 0, 0); // RED
  }
  client.stop();
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pixel.begin();
  setColor(50, 20, 0); // ORANGE: Booting

  // Network Setup (-1 prevents pin conflicts!)
  pinMode(ETH_CS, OUTPUT); 
  digitalWrite(ETH_CS, HIGH);
  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI, -1); 
  Ethernet.init(ETH_CS);
  
  Ethernet.begin(mac, staticIP, dns, gateway, subnet);

  unsigned long linkStart = millis();
  while (Ethernet.linkStatus() != LinkON && millis() - linkStart < 4000) {
    delay(100);
  }

  if (Ethernet.linkStatus() == LinkON) {
    Serial.print("\n[NET] Connected! IP: ");
    Serial.println(Ethernet.localIP());
    
    // Explicitly reset the Watchdog before starting the heavy lifting
    //esp_task_wdt_reset(); 
    runOTA();
  } else {
    Serial.println("\n[NET] Failed to detect Ethernet cable.");
    setColor(50, 0, 0); // RED
  }
}

void loop() {
  // Do nothing. We just want to see if setup() survives.
  delay(1000);
}