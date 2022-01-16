#ifndef DRIVEWAY_MONITOR_RADIO_H__
#define DRIVEWAY_MONITOR_RADIO_H__

#include "sx1276reg.h"
#include "sx1276.h"

#define SX1276_RESET_LOW_TIME 10
#define SX1276_RESET_WAIT_TIME 50

namespace driveway {

class Radio
{
private:
  bool sx1276Valid;
  uint8_t sx1276Version;
  SX1276Radio& radio;

public:
  bool isValid() const { return sx1276Valid; }
  void reset();
  void reconfigure();
  void setup();
  void printState();
  bool transmitPacket(const void *payload, byte len);

  Radio(SX1276Radio& sx1276Radio)
  : sx1276Valid(false),
    sx1276Version(0),
    radio(sx1276Radio)
  { }
};

void Radio::reset() {
  this->sx1276Valid = false;
  digitalWrite(PIN_SX1276_RST, LOW);
  delay(SX1276_RESET_LOW_TIME); // spec states to pull low 100us then wait at least 5 ms
  digitalWrite(PIN_SX1276_RST, HIGH);
  delay(SX1276_RESET_WAIT_TIME);
}

void Radio::reconfigure() {
  SPI.begin();
  // Just read this once, rather than continuously
  this->sx1276Version = this->radio.ReadVersion();
  if (!this->radio.Begin()) {
    Serial.println(F("Init Fault: SX1276"));
  } else {
    // Aim for improved (shorter) ToA
    this->radio.SetSpreadingFactor(7);
    this->radio.SetBandwidth(SX1276_LORA_BW_125000);
    this->radio.SetCarrier(920000000);
    this->sx1276Valid = true;
  }
  SPI.end();
}

void Radio::setup() {
  Serial.println(F("setup_radio"));
  this->sx1276Valid = false;
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
  this->reset();
  this->reconfigure();
}

void Radio::printState() {
  // Read settings directly out of the radio as a sanity check
  uint32_t carrier_hz = 0;
  byte sf = 0;
  SPI.begin();
  this->radio.ReadCarrier(carrier_hz);
  this->radio.ReadSpreadingFactor(sf);
  SPI.end();

  Serial.print(F("sx1276: ver=")); Serial.print(this->sx1276Version);
  Serial.print(F(" fault=")); Serial.print(!this->sx1276Valid);
  Serial.print(F(" tune=")); Serial.print(carrier_hz);
  Serial.print(F(" sf=")); Serial.print(sf);
  Serial.print(F(" toa(13)=")); Serial.print(this->radio.PredictTimeOnAir(13));
  Serial.println();
}

bool Radio::transmitPacket(const void *payload, byte len) {
  bool ok = false;
  SPI.begin();
  if (!this->radio.TransmitMessage(payload, len, false)) {
    // TX TIMEOUT - interrupt bit not set by the predicted toa...
    this->radio.Standby(); // in case...
  } else {
    // Radio.Standby on success should not be required...
    ok = true;
  }
   SPI.end();
  return ok;
}

}

#endif
