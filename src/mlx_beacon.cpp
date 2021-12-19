#include <Arduino.h>
#include <MLX90393.h>
#include <elapsedMillis.h>
#include <SPI.h>
#include "sx1276.h"

#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

#if defined(XMC_BOARD)
#define PIN_LED4         LED1
#define PIN_SX1276_RST  4
#define PIN_SX1276_CS   3
#define PIN_SX1276_MISO 0
#define PIN_SX1276_MOSI 1
#define PIN_SX1276_SCK  2
#define ICACHE_FLASH_ATTR
#define BOARD_NAME STRINGIFY(XMC_BOARD)
#else
#error "Unsupported configuration"
#endif

MLX90393 mlx;
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // double check
SX1276Radio radio(PIN_SX1276_CS, spiSettings);

bool mlxStartedOk = false;
bool radioStartedOk = false;

elapsedMillis elapsedTime;
uint16_t minDelay = 0;

void delayAtBoot() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(4000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
}

void flashFaultLed(int faultCode) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
}

void setupMlx() {
  if (MLX90393::STATUS_OK != mlx.begin(0, 0)) {
    Serial.println(F("Error starting MLX90393"));
  } else {
    mlx.reset(); // beware, this changes defaults from begin()
    mlx.setGainSel(7);
    mlx.setResolution(0, 0, 0);
    mlx.setOverSampling(3); // increases mindelay
    mlx.setTemperatureCompensation(0);
    mlx.setDigitalFiltering(5); // reduces mindelay
    minDelay = mlx.convDelayMillis();
    Serial.print("MLX90393.minDelay="); Serial.println(minDelay);
    mlxStartedOk = true;
  }
}

void resetRadio() {
  digitalWrite(PIN_SX1276_RST, LOW);
  delay(10); // spec states to pull low 100us then wait at least 5 ms, so 10ms then 50ms should be ample
  digitalWrite(PIN_SX1276_RST, HIGH);
  delay(50);
}

void setupRadio() {
  radioStartedOk = false;
  pinMode(PIN_SX1276_RST,  OUTPUT);
  pinMode(PIN_SX1276_CS,   OUTPUT);
  digitalWrite(PIN_SX1276_CS, HIGH);
  digitalWrite(PIN_SX1276_RST, HIGH);
  digitalWrite(PIN_SX1276_MISO, HIGH);
  digitalWrite(PIN_SX1276_MOSI, HIGH);
  digitalWrite(PIN_SX1276_SCK,  HIGH);
  resetRadio();
  // init SPI and then program the chip to LoRa mode
  SPI.begin();
  Serial.print(F("SX1276: version=")); Serial.println(radio.ReadVersion());
  if (!radio.Begin()) {
    Serial.println(F("SX1276 init error"));
  } else {
    radio.SetCarrier(919000000);
    uint32_t carrier_hz = 0;
    radio.ReadCarrier(carrier_hz);
    Serial.print(F("Carrier: ")); Serial.println(carrier_hz);
    Serial.print(F("Predict ToA for 16 bytes:")); Serial.println(radio.PredictTimeOnAir(16));
    radioStartedOk = true;
  }
  SPI.end();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  delayAtBoot();
  Serial.begin(115200);
  Serial.println(F("Magnetic Field Disruption Probe"));
  Serial.print(F("Device: "));
  Serial.println(F(BOARD_NAME));
  setupMlx();
  setupRadio();
  elapsedTime = 0;
}

bool seenStatus0 = false;
elapsedMillis lastMlx;

void loop() {
  uint8_t status;

  bool faultMlx = false;
  bool faultRadio = !radioStartedOk;

  status = mlx.nop();
  if ( MLX90393::STATUS_ERROR == status) {
    Serial.println(F("Error reading MLX90393"));
    faultMlx = true;
  } else if (!seenStatus0) {
    char buf[256];
    snprintf(buf, sizeof(buf), "Initial MLX90393 status=%02x (%c) computed minDelay=%d", status, (status & MLX90393::ERROR_BIT ? '!':' '), minDelay);
    Serial.println(buf);
    seenStatus0 = true;
    lastMlx = 0;
  }

  // TODO: interleave transmit between start and read, so that
  // the ToA over compensates for the minDelay

  if (!faultMlx) {
    // Serial.println("Read mlx...");
    MLX90393::txyzRaw raw;
    status = mlx.startMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG);
    if (status & MLX90393::ERROR_BIT) {
      Serial.println(F("Error starting MLX90393 measure"));
    }
    if (minDelay < 1) { delayMicroseconds(600); } else { delay(minDelay+10); }
    status = mlx.readMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG, raw);
    if (status & MLX90393::ERROR_BIT) {
      Serial.println(F("Error reading MLX90393 measure"));
    }

    const MLX90393::txyz values = mlx.convertRaw(raw);
    byte xxor = 0;
    if (!faultRadio) {
      char msg[256];
      // We need to get length down to reduce ToA
      snprintf(msg, sizeof(msg), "%d,%d,%d,%d,", (int)values.x, (int)values.y, (int)values.z, (int)values.t);
      int n = 0;
      for (n=0; msg[n]; n++) {
        xxor = xxor ^ (byte)msg[n];
      }
      snprintf(msg+n, sizeof(msg)-n, "%02x%02x", status, xxor);
      SPI.begin();
      radio.TransmitMessage(msg, strlen(msg)+1);
      radio.Standby();
      SPI.end();
    }

    char buf[256];
    snprintf(buf, sizeof(buf), "STATUS %02x (%c) DATA.xyzt %d,%d,%d,%d minDelay=%d since=%lu xxor=%02x",
        status, (status & MLX90393::ERROR_BIT ? '!':' '),
        (int)values.x, (int)values.y, (int)values.z, (int)values.t, minDelay, (long)elapsedTime, xxor);
    Serial.println(buf);
  }
  if (faultMlx) {
    flashFaultLed(1);
    delay(3000);
  }
  if (faultMlx && faultRadio) {
    delay(3000);
  }
}
