#include <Arduino.h>
#include <MLX90393.h>
#include <elapsedMillis.h>
#include <SPI.h>
#include "sx1276reg.h"
#include "sx1276.h"

// Sanity check our pio settings, because unfortunately this affects arduino main cpp
#ifdef SERIAL_DEBUG
#ifdef SERIAL_ONBOARD
#error oops1
#endif
#ifdef SERIAL_HOSTPC
#error oops2
#endif
#endif

#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

#if defined(XMC_BOARD)
#define PIN_LED_MAIN    LED_BUILTIN   // Arduino 14
#define PIN_LED_XTRA    LED2          // Arduino 15

#define PIN_SX1276_RST  4  // hw 5
#define PIN_SX1276_CS   3  // hw 4
#define PIN_SX1276_MISO 0  // hw 1
#define PIN_SX1276_MOSI 1  // hw 2
#define PIN_SX1276_SCK  2  // hw 3

#define PIN_SDA 11         // hw 11
#define PIN_SCL 10         // hw 12

#define MLX_IRQ 9          // hw 15
#define PWM_OUT 8          // hw 16

// 5 == GPIO     // hw 6
// 6 = UART TX   // hw 7
// 7 == UART RX  // hw 8
// 13 == A0      // hw 9
// 12 == A1      // hw 10

#define ICACHE_FLASH_ATTR
#define BOARD_NAME STRINGIFY(XMC_BOARD)
#else
#error "Unsupported configuration"
#endif

#define PIN_LED_TRANSMIT PIN_LED_XTRA

#define SX1276_RESET_LOW_TIME 10
#define SX1276_RESET_WAIT_TIME 50

#define SERIAL_BOOT_DELAY_MS 2500
#define SHORT_LED_FLASH_MS 150

#define MEASURE_FLAGS (MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG)

// Set this to true to emulate driveway1, and transmit all frames on radio, in packet compatible with driveway1
bool modeDebugRadioAllMeasurements = false;

// Set this to true, to print all frames to serial
bool modeDebugSerialAllMeasurements = false;

// Set this to 0, to print all frames, otherwise, 10 to print every 10th, to emulate driveway1, etc.
int debugSerialSampleFrameInterval = 10;

static SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
static SX1276Radio Radio(PIN_SX1276_CS, spiSettings, true);
static MLX90393 MlxSensor;

struct RadioStatus_t
{
  bool sx1276Valid;
  byte sx1276Version;
  RadioStatus_t()
  : sx1276Valid(false),
    sx1276Version(0)
  { }
};

struct MlxStatus_t
{
  bool mlxValid;
  byte lastNopCode;
  int resetCount;

  uint16_t minDelayHeuristic;
  
  elapsedMillis lastRequest;
  bool measurePending;
  
  bool measureValid;
  long lastMeasureValid;

  bool mlxRequestError;
  bool mlxReadError;
  long allFrameCounter;
  MLX90393::txyz values;
  float magnitude;

  MlxStatus_t()
  : mlxValid(false),
    lastNopCode(0),
    resetCount(0),
    minDelayHeuristic(0),
    measurePending(false),
    measureValid(false),
    lastMeasureValid(0),
    mlxRequestError(false),
    mlxReadError(false),
    allFrameCounter(0),
    magnitude(0.F)
  { }
};

RadioStatus_t RadioStatus;
MlxStatus_t MlxStatus;

elapsedMillis uptime;

void led_reset() {
  digitalWrite(PIN_LED_MAIN, LOW);
  digitalWrite(PIN_LED_XTRA, LOW);
}

void led_short_flash(uint8_t pin = PIN_LED_MAIN) {
  digitalWrite(pin, HIGH);
  delay(SHORT_LED_FLASH_MS);
  digitalWrite(pin, LOW);
}

// Used at end of boot when serial port is ready, and a second time after first background reading
void led_five_short_flash() {
  for (int n=0; n < 5; n++) {
    delay(SHORT_LED_FLASH_MS);
    led_short_flash();
  }
}

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
  auto radioVersion = Radio.ReadVersion();
  if (!Radio.Begin()) {
    Serial.println(F("Init Fault: SX1276"));
  } else {
    // Aim for improved (shorter) ToA
    Radio.SetSpreadingFactor(7);
    Radio.SetCarrier(920000000);
    Radio.SetBandwidth(SX1276_LORA_BW_125000);
    RadioStatus.sx1276Valid = true;
  }
  SPI.end();
}

void setup_radio() {
  RadioStatus.sx1276Valid = false;
  pinMode(PIN_SX1276_RST,  OUTPUT);
  pinMode(PIN_SX1276_CS,   OUTPUT);
  digitalWrite(PIN_SX1276_MISO, HIGH);
  digitalWrite(PIN_SX1276_MOSI, HIGH);
  digitalWrite(PIN_SX1276_SCK,  HIGH);
  digitalWrite(PIN_SX1276_CS, HIGH);
  digitalWrite(PIN_SX1276_RST, HIGH);
  reset_radio();
  configure_radio();
}

void configure_mlx() {
  MlxSensor.reset(); // beware, this changes defaults from begin()
  MlxSensor.setGainSel(7);
  MlxSensor.setResolution(0, 0, 0);
  MlxSensor.setOverSampling(3); // increases mindelay
  MlxSensor.setTemperatureCompensation(0);
  MlxSensor.setDigitalFiltering(5); // reduces mindelay
  MlxStatus.lastNopCode = MlxSensor.nop();
  auto minDelay = MlxSensor.convDelayMillis();
  MlxStatus.minDelayHeuristic = (minDelay < 1) ? 1 : minDelay + 10;
  MlxStatus.mlxValid = true;
}

static void setup_mlx() {
  MlxStatus.mlxValid = false;
  if (MLX90393::STATUS_OK != MlxSensor.begin(0, 0)) {
    MlxStatus.lastNopCode = MlxSensor.nop();
    Serial.println(F("Init Fault: MLX"));
  } else {
    configure_mlx();
  }
}

void print_radio_state() {
  // Read settings directly out of the radio as a sanity check
  uint32_t carrier_hz = 0;
  byte sf = 0;
  SPI.begin();
  Radio.ReadCarrier(carrier_hz);
  Radio.ReadSpreadingFactor(sf);
  SPI.end();

  Serial.print(F("sx1276: ver=")); Serial.print(RadioStatus.sx1276Version);
  Serial.print(F(" fault=")); Serial.print(!RadioStatus.sx1276Valid);
  Serial.print(F(" tune=")); Serial.print(carrier_hz);
  Serial.print(F(" sf=")); Serial.print(sf);
  Serial.print(F(" toa(13)=")); Serial.print(Radio.PredictTimeOnAir(13));
  Serial.println();
}

void print_mlx_state() {
  uint8_t gainSel, hall, osr, rx, ry, rz, df;
  MlxSensor.getGainSel(gainSel);
  MlxSensor.getHallConf(hall);
  MlxSensor.getOverSampling(osr);
  MlxSensor.getResolution(rx, ry, rz);
  MlxSensor.getDigitalFiltering(df);
  byte nop = MlxSensor.nop();

  Serial.print(F("MLX90393:"));
  Serial.print(F(" minDelay'=")); Serial.print(MlxStatus.minDelayHeuristic); 
  Serial.print(F(" gainSel=")); Serial.print(gainSel);
  Serial.print(F(" hall=")); Serial.print(hall);
  Serial.print(F(" osr=")); Serial.print(osr);
  Serial.print(F(" filter=")); Serial.print(df);
  Serial.print(F(" nop=")); Serial.print(nop);
  Serial.println();
}

void setup() {
  pinMode(PIN_LED_MAIN, OUTPUT);
  pinMode(PIN_LED_XTRA, OUTPUT);
  digitalWrite(PIN_LED_MAIN, HIGH);
  digitalWrite(PIN_LED_XTRA, HIGH);

  delay(SERIAL_BOOT_DELAY_MS);
  led_reset();
  led_five_short_flash();  

  Serial.begin(115200);
  Serial.println(F("SentriFarm Magnetic Field Disruption Probe"));
  Serial.print(F("Device: "));
  Serial.println(F(BOARD_NAME));

  setup_radio();
  print_radio_state();

  setup_mlx();
  print_mlx_state();
}

bool measureMlxRequestIf() {
  if (MlxStatus.measurePending) {
    return false;
  }
  // No timing constraint on how soon after a reading we can request again...
  MlxStatus.lastRequest = 0;
  MLX90393::txyzRaw raw;
  auto status = MlxSensor.startMeasurement(MEASURE_FLAGS);
  if (status & MLX90393::ERROR_BIT) {
    MlxStatus.mlxRequestError = true;
    MlxStatus.lastNopCode = MlxSensor.nop();
    Serial.println(F("MLX request error"));
  } else {
    MlxStatus.measurePending = true;
  }
  return true;
}

bool measureMlxIf() {
  if (MlxStatus.mlxRequestError) {
    return false;
  }
  if (MlxStatus.measurePending && (MlxStatus.lastRequest >= MlxStatus.minDelayHeuristic)) {
    MLX90393::txyzRaw raw;
    MlxStatus.measureValid = false;
    auto status = MlxSensor.readMeasurement(MEASURE_FLAGS, raw);
    if (status & MLX90393::ERROR_BIT) {
      MlxStatus.mlxReadError = true;
      MlxStatus.lastNopCode = MlxSensor.nop();
      Serial.println(F("MLX read error"));
    } else {
      MlxStatus.lastMeasureValid = uptime;
      MLX90393::txyz& values = MlxStatus.values = MlxSensor.convertRaw(raw);
      MlxStatus.magnitude = sqrt(values.x * values.x + values.y * values.y + values.z * values.z);
      MlxStatus.measureValid = true;
    }
    MlxStatus.measurePending = false;
    return true;
  }
  return false;
}

bool transmitPacket(const void *payload, byte len) {
  bool ok = false;
  digitalWrite(PIN_LED_TRANSMIT, HIGH);
  SPI.begin();
  if (!Radio.TransmitMessage(payload, len, false)) {
    // TX TIMEOUT - interrupt bit not set by the predicted toa...
    Radio.Standby(); // in case...
  } else {
    // Radio.Standby on success should not be required...
    ok = true;
  }
  SPI.end();
  digitalWrite(PIN_LED_TRANSMIT, LOW);
  return ok;
}

void transmitErrorBeacon() {
  char packet[14];
  // send trailing space as a defensive hack against intermittent bug where last byte not reeived
  snprintf(packet, sizeof(packet), "FAULT,%02x,%d,%d ", MlxStatus.lastNopCode, MlxStatus.mlxRequestError, MlxStatus.mlxReadError);
  SPI.begin();
  if (!Radio.TransmitMessage(packet, strlen(packet), false)) {
    // TX TIMEOUT - interrupt bit not set by the predicted toa...
  }
  Radio.Standby();
  SPI.end();
}

void transmitDebugCollectionFrame() {
  static long counter = 0;
  byte packet[13];
  const long tEvent = MlxStatus.lastMeasureValid / 10;
  const uint16_t m = uint16_t(MlxStatus.magnitude);
  const int t = MlxStatus.values.t;
  packet[0] = 12;
  packet[1] = 0x5f;  // not really sF, but hey
  packet[2] = 1;     // this type of message
  packet[3] = (counter >> 8) & 0xff; // auto wrap counter @ 65535 packets, just useful for detecting skip
  packet[4] = (counter & 0xff);
  packet[5] = tEvent & 0xff;
  packet[6] = (tEvent >> 8) & 0xff;
  packet[7] = (tEvent >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  packet[8] = m & 0xff;
  packet[9] = (m >> 8) & 0xff;
  // Pack temp into 6 bits. We cant handle negative for now, but ths is for development anyway
  packet[10] = (t & 0x3f) | ((MlxStatus.resetCount & 0x3) << 6);
  packet[11] = 0;
  packet[12] = 0;
  transmitPacket(packet, 13);
  counter ++;
}

void printDebugCollectionFrame() {
  auto now = uptime;
  Serial.print(now);
  Serial.print(F(",mlx"));
  Serial.print(','); Serial.print(MlxStatus.allFrameCounter);
  Serial.print(','); Serial.print(MlxStatus.mlxValid);
  Serial.print(','); Serial.print(MlxStatus.mlxRequestError);
  Serial.print(','); Serial.print(MlxStatus.mlxReadError);
  Serial.print(','); Serial.print(MlxStatus.lastRequest - uptime);
  Serial.print(','); Serial.print(MlxStatus.lastNopCode);
  Serial.print(','); Serial.print(MlxStatus.values.x);
  Serial.print(','); Serial.print(MlxStatus.values.y);
  Serial.print(','); Serial.print(MlxStatus.values.z);
  Serial.print(','); Serial.print(MlxStatus.values.t);
  Serial.print(','); Serial.print(MlxStatus.magnitude);
  Serial.print(F(",timing"));
  Serial.println();
}

bool debugRadioTransmitPending = false;
elapsedMillis lastErrorBeacon;

void loop() {
  // We need to interleave requesting and reading the MLX results with transmitting the previous result
  // because both have a time to conclusion

  // This will return true if a measurement was requested (or an error occurred) - which it will be if none is pending
  // the question is, should there be a minimum interval between reading and next requests (so far, it appears not)
  bool priorRequestError = MlxStatus.mlxRequestError;
  measureMlxRequestIf();

  // This will transmit the latest state, at full bore, as long as last request (not reading) succeeded
  // measureValid will be false if request succeeded then reading failed
  if (modeDebugRadioAllMeasurements && MlxStatus.measureValid && debugRadioTransmitPending) {
    transmitDebugCollectionFrame();
  }
  debugRadioTransmitPending = false;

  // This will return true whether reading completed or an error occurred
  bool priorReadError = MlxStatus.mlxReadError;
  if (measureMlxIf()) {
    MlxStatus.allFrameCounter ++;
    debugRadioTransmitPending = true;

    // print debug to serial, either continuous, or on a sample if we have a constrained debug link like an ESP01
    if (modeDebugSerialAllMeasurements) {
      if ((debugSerialSampleFrameInterval < 1) || (MlxStatus.allFrameCounter % debugSerialSampleFrameInterval == 0)) {
        printDebugCollectionFrame();
      }
    }
  }

  bool newFault = false;
  if (MlxStatus.mlxRequestError && priorRequestError != MlxStatus.mlxRequestError) {
    // we just had a request fail start
    Serial.println(F("MLX request fault detect"));
    newFault = true;
  }
  if (MlxStatus.mlxReadError && priorReadError != MlxStatus.mlxReadError) {
    // we just had a read fail start
    Serial.println(F("MLX read fault detect"));
    newFault = true;
  }
  // if we have an error state, transmit an error beacon at 5 second intervals
  if (newFault || lastErrorBeacon > 5000) {
    lastErrorBeacon = 0;
    transmitErrorBeacon();
    Serial.print(F("Fault"));
    Serial.print(','); Serial.print(MlxStatus.lastNopCode, HEX);
    Serial.print(','); Serial.print(MlxStatus.mlxRequestError, HEX);
    Serial.print(','); Serial.print(MlxStatus.mlxReadError, HEX);
    Serial.println();
  }
}