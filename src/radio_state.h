#ifndef DRIVEWAY_MONITOR_RADIO_STATE_H__
#define DRIVEWAY_MONITOR_RADIO_STATE_H__

#include "sx1276reg.h"
#include "sx1276.h"

#define SX1276_RESET_LOW_TIME 10
#define SX1276_RESET_WAIT_TIME 50

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
const bool inAir9b = true;
SX1276Radio Radio(PIN_SX1276_CS, spiSettings, inAir9b);

struct RadioStatus_t
{
  bool sx1276Valid;
  byte sx1276Version;
  RadioStatus_t()
  : sx1276Valid(false),
    sx1276Version(0)
  { }
};

RadioStatus_t RadioStatus;

void reset_radio() {
  RadioStatus.sx1276Valid = false;
  digitalWrite(PIN_SX1276_RST, LOW);
  delay(SX1276_RESET_LOW_TIME); // spec states to pull low 100us then wait at least 5 ms
  digitalWrite(PIN_SX1276_RST, HIGH);
  delay(SX1276_RESET_WAIT_TIME);
}

void configure_radio() {
  SPI.begin();
  // Just read this once, rather than continuously
  RadioStatus.sx1276Version = Radio.ReadVersion();
  if (!Radio.Begin()) {
    Serial.println(F("Init Fault: SX1276"));
  } else {
    // Aim for improved (shorter) ToA
    Radio.SetSpreadingFactor(7);
    Radio.SetBandwidth(SX1276_LORA_BW_125000);
    Radio.SetCarrier(920000000);
    RadioStatus.sx1276Valid = true;
  }
  SPI.end();
}

void setup_radio() {
  Serial.println(F("setup_radio"));
  RadioStatus.sx1276Valid = false;
  pinMode(PIN_SX1276_RST,  OUTPUT);
  pinMode(PIN_SX1276_CS,   OUTPUT);
#if defined(TEENSYDUINO)
  SPI.setSCK(PIN_SX1276_SCK);
#endif
  digitalWrite(PIN_SX1276_CS, HIGH);
  digitalWrite(PIN_SX1276_RST, HIGH);
  digitalWrite(PIN_SX1276_MISO, HIGH);
  digitalWrite(PIN_SX1276_MOSI, HIGH);
  digitalWrite(PIN_SX1276_SCK,  HIGH);
  reset_radio();
  configure_radio();
}

#endif
