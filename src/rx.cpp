#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <elapsedMillis.h>
#include "sx1276.h"

#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

// Supports the following configurations:
//
// (1) XMC2GO XMC1100
//     MISO, ardunio pin #0, XMC p0.6
//     MOSI, ardunio pin #1, XMC p0.7
//     SCK, ardunio pin #2, XMC p0.8
//     CS, ardunio pin #3, XMC p0.9
//     RST, ardunio pin #4, XMC p0.14
//     LED4, onboard LED == LED1
//
// (2) ESP8266
//     GPIO12 = MISO (HSPI)
//     GPIO13 = MOSI (HSPI)
//     GPIO14 = SCK (HSPI)
//     GPIO15 = CS
//     GPIO0  = RST
//     GPIO2  = LED4 (inverted)
//

#if defined(ESP8266)
#include <c_types.h>
#include <Esp.h> // deep sleep
#define PIN_LED4         4
#define PIN_SX1276_RST   0
#define PIN_SX1276_CS   15
#define PIN_SX1276_MISO 12
#define PIN_SX1276_MOSI 13
#define PIN_SX1276_SCK  14
#elif defined(TEENSYDUINO)
#define PIN_LED4         5
#define PIN_SX1276_RST  21
#define PIN_SX1276_CS   10
#define PIN_SX1276_MISO 12
#define PIN_SX1276_MOSI 11
#define PIN_SX1276_SCK  14
#define ICACHE_FLASH_ATTR
#elif defined(XMC_BOARD)
#define PIN_LED4         LED1
#define PIN_SX1276_RST  4
#define PIN_SX1276_CS   3
#define PIN_SX1276_MISO 0
#define PIN_SX1276_MOSI 1
#define PIN_SX1276_SCK  2
#define ICACHE_FLASH_ATTR
#else
#error "Unsupported configuration"
#endif

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // double check

SX1276Radio radio(PIN_SX1276_CS, spiSettings);

bool started_ok = false;

ICACHE_FLASH_ATTR
void setup()
{
  delay(1000); // let last of ESP8266 junk get past
  Serial.begin(115200);
  Serial.println();

  Serial.print(F("\n\n\nSentrifarm : sx1276 beacon : "));
#if defined(TEENSYDUINO)
  Serial.println(F("TEENSY-LC"));
#elif defined(ESP8266)
  Serial.println(F("ESP8266 ESP-201"));
#elif defined(XMC_BOARD)
  Serial.println(F(STRINGIFY(XMC_BOARD)));
#endif

  pinMode(PIN_LED4,        OUTPUT);
  pinMode(PIN_SX1276_RST,  OUTPUT);
  pinMode(PIN_SX1276_CS,   OUTPUT);

  // Power on the LED (active low2)
  digitalWrite(PIN_LED4, LOW);

#if defined(TEENSYDUINO)
  SPI.setSCK(PIN_SX1276_SCK);
#endif

  digitalWrite(PIN_SX1276_CS, HIGH);
  digitalWrite(PIN_SX1276_RST, HIGH);
  digitalWrite(PIN_SX1276_MISO, HIGH);
  digitalWrite(PIN_SX1276_MOSI, HIGH);
  digitalWrite(PIN_SX1276_SCK,  HIGH);

  // Reset the sx1276 module
  digitalWrite(PIN_SX1276_RST, LOW);
  delay(10); // spec states to pull low 100us then wait at least 5 ms
  digitalWrite(PIN_SX1276_RST, HIGH);
  delay(50);

  // init SPI and then program the chip to LoRa mode
  SPI.begin();
  Serial.print(F("SX1276: version=")); Serial.println(radio.ReadVersion());
  if (!radio.Begin()) {
    Serial.println(F("SX1276 init error"));
    // TODO: flash the LED
  } else {
    radio.SetCarrier(919000000);
    uint32_t carrier_hz = 0;
    radio.ReadCarrier(carrier_hz);
    Serial.print(F("Carrier: ")); Serial.println(carrier_hz);
    started_ok = true;
  }
  SPI.end();
  delay(500);
}

void go_to_sleep(int ms)
{
#if defined(ESP8266)
  ESP.deepSleep(ms * 1000, WAKE_RF_DISABLED); // long enough to see current fall on the USB power meter
  delay(500); // Note, deep sleep actually takes a bit to activate, so we want to make sure loop doesnt do anything...
#else
  delay(ms);
#endif
}

void double_short()
{
  digitalWrite(PIN_LED4, HIGH);
  delay(250);
  digitalWrite(PIN_LED4, LOW);
  delay(250);
  digitalWrite(PIN_LED4, HIGH);
  delay(250);
  digitalWrite(PIN_LED4, LOW);
  delay(250);
  digitalWrite(PIN_LED4, HIGH);
}

bool got_message = false;

int rx_count = 0;
int crc_count = 0;
int timeout_count = 0;

elapsedMillis timeElapsed;


void loop() {
  if (!started_ok) {
    digitalWrite(PIN_LED4, LOW);
    delay(500);
    digitalWrite(PIN_LED4, HIGH);
    delay(500);
    return;
  }

  SPI.begin();

  // If we get symbol timeout then double short flash and try again
  // If we get CRC then single longer flash
  // otherwise, latch LED on
  if (!got_message) { double_short(); }

  byte buffer[128];
  bool crc_error = false;
  byte received = 0;
  elapsedMillis trx;
  if (radio.ReceiveMessage(buffer, sizeof(buffer), received, crc_error))
  {
    const int N=16;
    byte r=0;
    char buf[4];
    for (int line=0; line < (received+N-1)/N; line++) {
      snprintf(buf, sizeof(buf), "%02x ", r); Serial.print(buf);
      byte r2 = r;
      for (int c=0; c < N; c++, r++) {
        if (r < received) {
          snprintf(buf, sizeof(buf), "%02x ", buffer[r]); Serial.print(buf);
        } else { Serial.print(F("   ")); }
      }
      for (int c=0; c < N; c++, r2++) {
        if (r2 < received) {
          Serial.print(char(buffer[r2]));
        } else { Serial.print(' '); }
      }
      Serial.println();
    }
    got_message = true;
    rx_count ++;
    digitalWrite(PIN_LED4, LOW);
  } else if (crc_error) {
    crc_count ++;
    Serial.println("CRC\n");
    digitalWrite(PIN_LED4, HIGH);
    delay(600);
    digitalWrite(PIN_LED4, LOW);
    delay(600);
    digitalWrite(PIN_LED4, HIGH);
  } else {
    // Serial.println("timeout"); Serial.println(trx); Serial.println();
    timeout_count ++;
  }
  if (timeElapsed > 60000)  {
    Serial.print("Received message count: "); Serial.print(rx_count);
    Serial.print(" Timeout count: "); Serial.print(timeout_count);
    Serial.print(" CRC count: "); Serial.print(crc_count);
    Serial.println();
    timeElapsed = 0;
  }
  SPI.end();  
}
