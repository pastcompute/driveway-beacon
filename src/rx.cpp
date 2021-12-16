#define ENABLE_SPI
// #undef ENABLE_SPI

#include <Arduino.h>
#if defined(ENABLE_SPI)
#include <Wire.h>
#include <SPI.h>
#include "elapsedMillis.h"
#include "sx1276.h"
#endif

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
#define ESP8266_WHITE8
#include <c_types.h>
#include <Esp.h> // deep sleep
#define PIN_SX1276_RST   0
#ifdef ESP8266_WHITE8
#define PIN_SX1276_CS    2
#define PIN_LED4         15 // FFS the serial port shares LED_BUILTIN on the white board
#else
#define PIN_SX1276_CS    15
#define PIN_LED4         4
#endif
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
#define PIN_LED4         LED_BUILTIN
#define PIN_SX1276_RST  4
#define PIN_SX1276_CS   3
#define PIN_SX1276_MISO 0
#define PIN_SX1276_MOSI 1
#define PIN_SX1276_SCK  2
#define ICACHE_FLASH_ATTR
#else
#error "Unsupported configuration"
#endif

#define LED_ON HIGH
#define LED_OFF LOW
#if defined(ESP8266_WHITE8)
#elif defined(XMC_BOARD)
#else
#define LED_ON LOW
#define LED_OFF HIGH
#endif

#if defined(ENABLE_SPI)
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // double check

SX1276Radio radio(PIN_SX1276_CS, spiSettings);
#endif

bool started_ok = false;

ICACHE_FLASH_ATTR
void setup()
{
  pinMode(PIN_LED4,        OUTPUT);
  digitalWrite(PIN_LED4, LED_ON);
#if defined XMC_BOARD
  pinMode(LED2,        OUTPUT);
  digitalWrite(LED2, LED_ON);
  delay(4000);
  digitalWrite(PIN_LED4, LED_OFF);
#endif
  delay(2500); // let last of ESP8266 junk get past
  digitalWrite(PIN_LED4, LED_OFF);
  Serial.begin(115200);
  delay(100);
  Serial.println();
  delay(100);
  Serial.println();
  delay(100);

  Serial.print(F(".\n\r.\n\r.\n\rSentrifarm : sx1276 sniffer : "));
#if defined(TEENSYDUINO)
  Serial.println(F("TEENSY-LC"));
#elif defined(ESP8266_WHITE8)
  Serial.println(F("ESP8266 Generic 16 pin breakout"));
#elif defined(ESP8266)
  Serial.println(F("ESP8266 ESP-201"));
#elif defined(XMC_BOARD)
  Serial.println(F(STRINGIFY(XMC_BOARD)));
#endif


  pinMode(PIN_SX1276_RST,  OUTPUT);
  pinMode(PIN_SX1276_CS,   OUTPUT);
#if defined(ENABLE_SPI)
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
  delay(20); // spec states to pull low 100us then wait at least 5 ms
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
#endif
}

#if 0
void go_to_sleep(int ms)
{
#if defined(ESP8266)
  ESP.deepSleep(ms * 1000, WAKE_RF_DISABLED); // long enough to see current fall on the USB power meter
  delay(500); // Note, deep sleep actually takes a bit to activate, so we want to make sure loop doesnt do anything...
#else
  delay(ms);
#endif
}
#endif

void double_short()
{
  digitalWrite(PIN_LED4, LED_ON);
  delay(250);
  digitalWrite(PIN_LED4, LED_OFF);
  delay(250);
  digitalWrite(PIN_LED4, LED_ON);
  delay(250);
  digitalWrite(PIN_LED4, LED_OFF);
  delay(250);
  digitalWrite(PIN_LED4, LED_ON);
}

bool got_message = false;

int rx_count = 0;
int crc_count = 0;
int timeout_count = 0;

#if defined(ENABLE_SPI)
elapsedMillis timeElapsed;
#endif


void loop() {
  if (!started_ok) {
    digitalWrite(PIN_LED4, LED_OFF);
    delay(500);
    digitalWrite(PIN_LED4, LED_ON);
    delay(500);
    digitalWrite(PIN_LED4, LED_OFF);
    delay(500);
    digitalWrite(PIN_LED4, LED_ON);
    delay(500);
    digitalWrite(PIN_LED4, LED_OFF);
    delay(2500);
    Serial.println(F("no start"));
    return;
  }

  // If we get symbol timeout then double short flash and try again
  // If we get CRC then single longer flash
  // otherwise, latch LED on
  if (!got_message) { double_short(); }

#if defined(ENABLE_SPI)
  SPI.begin();

  byte buffer[128];
  bool crc_error = false;
  byte received = 0;
  elapsedMillis trx;
  memset(buffer, 0, sizeof(buffer));
  if (radio.ReceiveMessage(buffer, sizeof(buffer), received, crc_error))
  {
    radio.Standby(); // hmmm turns out we had to do this in line 325 of leaf.info; the 'node' version does it after transmit
    // Because this library is using single mode... (it seems didn't port continuous mode into it...)
    Serial.print(F("Rx "));
    Serial.print(received);
    Serial.print(F(" "));
    Serial.print(crc_error);
    Serial.println();
    const int N=16;
    int r=0;
    char buf[6];
    for (int line=0; line < (received+N-1)/N; line++) {
      // print the byte address of the start of each row first
      snprintf(buf, sizeof(buf), "%02x: ", r); Serial.print(buf);
      int r2 = r;
      for (int c=0; c < N; c++, r++) {
        if (r < received) {
          snprintf(buf, sizeof(buf), "%02x ", buffer[r]); Serial.print(buf);
        } else { Serial.print(F("   ")); }
      }
      Serial.print(F("    "));
      for (int c=0; c < N; c++, r2++) {
        if (r2 < received) {
          char v = buffer[r2];
          Serial.print((v >=32 && v <=127) ? v : ' ');
        } else { Serial.print('.'); }
      }
      Serial.println();
    }
    got_message = true;
    rx_count ++;
    digitalWrite(PIN_LED4, LED_OFF);
  } else if (crc_error) {
    radio.Standby();
    crc_count ++;
    Serial.println("CRC\n");
    digitalWrite(PIN_LED4, LED_ON);
    delay(600);
    digitalWrite(PIN_LED4, LED_OFF);
    delay(600);
    digitalWrite(PIN_LED4, LED_ON);
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
#endif
}
