#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Update.h>
#include <Adafruit_NeoPixel.h>

// ---------------- CONFIG ----------------

const int CURRENT_VERSION = 10;

#define ETH_CS    9
#define ETH_SCK   36
#define ETH_MISO  37
#define ETH_MOSI  35

#define NEO_PIN   48

byte mac[] = {0xDE,0xAD,0xBE,0xEF,0xFE,0xED};

IPAddress staticIP(192,168,1,152);
IPAddress dns(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

IPAddress raspberryPi(192,168,1,239);
const int piPort = 5000;

// ----------------------------------------

EthernetClient client;
Adafruit_NeoPixel pixel(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- OTA ----------------

void runOTA()
{
  client.setTimeout(3000);

  if(!client.connect(raspberryPi, piPort))
    return;

  client.println("GET /firmware/version.txt HTTP/1.1");
  client.println("Host: 192.168.1.239");
  client.println("Connection: close");
  client.println();

  while(client.connected())
  {
    String line = client.readStringUntil('\n');
    if(line == "\r") break;
  }

  int availableVersion = client.readStringUntil('\n').toInt();
  client.stop();

  if(availableVersion <= CURRENT_VERSION)
    return;

  if(!client.connect(raspberryPi, piPort))
    return;

  client.println("GET /firmware/firmware.bin HTTP/1.1");
  client.println("Host: 192.168.1.239");
  client.println("Connection: close");
  client.println();

  long contentLength = 0;

  while(client.connected())
  {
    String line = client.readStringUntil('\n');
    line.trim();

    if(line.startsWith("Content-Length:"))
      contentLength = line.substring(15).toInt();

    if(line.length() == 0)
      break;
  }

  if(contentLength <= 0)
    return;

  if(!Update.begin(contentLength))
    return;

  uint8_t buffer[1024];
  size_t written = 0;

  while(client.connected() && written < contentLength)
  {
    size_t available = client.available();

    if(available)
    {
      int c = client.read(buffer, min(available,sizeof(buffer)));
      Update.write(buffer,c);
      written += c;
    }
  }

  if(Update.end() && Update.isFinished())
  {
    delay(1000);
    ESP.restart();
  }

  client.stop();
}

// ---------------- RGB FADE ----------------

void rgbFade()
{
  static uint8_t r=255,g=0,b=0;

  pixel.setPixelColor(0,pixel.Color(r,g,b));
  pixel.show();

  if(r>0 && b==0){r--; g++;}
  else if(g>0 && r==0){g--; b++;}
  else if(b>0 && g==0){b--; r++;}

  delay(30);
}

// ---------------- SETUP ----------------

void setup()
{
  Serial.begin(115200);

  pixel.begin();
  pixel.clear();
  pixel.show();

  pinMode(ETH_CS,OUTPUT);
  digitalWrite(ETH_CS,HIGH);

  SPI.begin(ETH_SCK,ETH_MISO,ETH_MOSI);
  Ethernet.init(ETH_CS);

  Ethernet.begin(mac,staticIP,dns,gateway,subnet);

  delay(2000);

  if(Ethernet.linkStatus()==LinkON)
    runOTA();
}

// ---------------- LOOP ----------------

void loop()
{
  rgbFade();
}