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

#define VERBOSE 1

#if VERBOSE
#include <stdio.h>
#define DEBUG(x ...) { char buf[128]; snprintf(buf, sizeof(buf), x); Serial.print(buf); }
#else
#define DEBUG(x ...)
#endif

#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

#if defined(XMC_BOARD)

#include <XMC1100.h>
#include <DeviceControlXMC.h>

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

XMCClass devCtrl;

#else
#error "Unsupported configuration"
#endif

#define PIN_LED_TRANSMIT PIN_LED_XTRA
#define PIN_LED_MLX PIN_LED_MAIN

#define SX1276_RESET_LOW_TIME 10
#define SX1276_RESET_WAIT_TIME 50

#define SERIAL_BOOT_DELAY_MS 3500
#define SHORT_LED_FLASH_MS 150

#define ERROR_BEACON_INTERVAL 1000
#define ERROR_BEACON_RESET_AFTER 5

#define MEASURE_FLAGS (MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG)

// Set this to true to emulate driveway1, and transmit all frames on radio, in packet compatible with driveway1
bool modeDebugRadioAllMeasurements = false;

// Set this to true, to print all frames to serial
bool modeDebugSerialAllMeasurements = false;

// Set this to 0, to print all frames, otherwise, 10 to print every 10th, to emulate driveway1, etc.
int debugSerialSampleFrameInterval = 0; //10;

#define HEARTBEAT_BEACON_MS 15000

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
const bool inAir9b = true;
SX1276Radio Radio(PIN_SX1276_CS, spiSettings, inAir9b);
MLX90393 MlxSensor;

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
  long returnTime;
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
    returnTime(0),
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

void led_short_flash(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delay(SHORT_LED_FLASH_MS);
  digitalWrite(pin, LOW);
}

// Used at end of boot when serial port is ready, and a second time after first background reading
void led_five_short_flash() {
  for (int n=0; n < 5; n++) {
    delay(SHORT_LED_FLASH_MS);
    led_short_flash(PIN_LED_MAIN);
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
  digitalWrite(PIN_SX1276_CS, HIGH);
  digitalWrite(PIN_SX1276_RST, HIGH);
  digitalWrite(PIN_SX1276_MISO, HIGH);
  digitalWrite(PIN_SX1276_MOSI, HIGH);
  digitalWrite(PIN_SX1276_SCK,  HIGH);
  reset_radio();
  configure_radio();
}

void configure_mlx() {
  Serial.println(F("configure_mlx"));
  pinMode(MLX_IRQ, INPUT_PULLDOWN);
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
  Serial.println(F("setup_mlx"));
  MlxStatus.mlxValid = false;
  if (MLX90393::STATUS_OK != MlxSensor.begin(0, 0, -1, Wire)) {
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
  auto mrq = digitalRead(MLX_IRQ);

  Serial.print(F("MLX90393:"));
  Serial.print(F(" minDelay'=")); Serial.print(MlxStatus.minDelayHeuristic); 
  Serial.print(F(" gainSel=")); Serial.print(gainSel);
  Serial.print(F(" hall=")); Serial.print(hall);
  Serial.print(F(" osr=")); Serial.print(osr);
  Serial.print(F(" filter=")); Serial.print(df);
  Serial.print(F(" nop=")); Serial.print(nop);
  Serial.print(F(" mrq=")); Serial.print(nop);
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
  // Throw in a short delay so background stuff keeps workig
  //delay(5);
  auto mrq = digitalRead(MLX_IRQ);
  int tc = 0;
#if defined(XMC_BOARD)
  tc = devCtrl.getTemperature();
#endif
  //DEBUG("measureMlxRequestIf %ld %d %d\n\r", (long)MlxStatus.lastRequest, mrq, tc);
  MLX90393::txyzRaw raw;
  MlxStatus.returnTime = MlxStatus.lastRequest;
  MlxStatus.lastRequest = 0;
  auto status = MlxSensor.startMeasurement(MEASURE_FLAGS);
  if (status & MLX90393::ERROR_BIT) {
    MlxStatus.mlxRequestError = true;
    MlxStatus.lastNopCode = status;
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
    auto mrq = digitalRead(MLX_IRQ);
    //DEBUG("measureMlxIf %ld %ld %u %d\n\r", (long)MlxStatus.lastRequest, (long)uptime, MlxStatus.minDelayHeuristic, mrq);
    MLX90393::txyzRaw raw;
    MlxStatus.measureValid = false;
    auto status = MlxSensor.readMeasurement(MEASURE_FLAGS, raw);
    if (status & MLX90393::ERROR_BIT) {
      MlxStatus.mlxReadError = true;
      MlxStatus.lastNopCode = status;
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
  // DEBUG("transmitDebugCollectionFrame %d\n\r", counter);
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
  Serial.print(','); Serial.print(MlxStatus.lastNopCode);
  Serial.print(','); Serial.print(MlxStatus.values.x);
  Serial.print(','); Serial.print(MlxStatus.values.y);
  Serial.print(','); Serial.print(MlxStatus.values.z);
  Serial.print(','); Serial.print(MlxStatus.values.t);
  Serial.print(','); Serial.print(MlxStatus.magnitude);
  Serial.print(F(",sx1276"));
  Serial.print(','); Serial.print(RadioStatus.sx1276Valid);
  Serial.print(F(",timing"));
  Serial.print(','); Serial.print(now - MlxStatus.lastRequest); // time of call to requestMeasurement
  Serial.print(','); Serial.print(MlxStatus.returnTime);        // interval between successive requests (eff. rate)
  Serial.print(','); Serial.print(MlxStatus.lastMeasureValid);  // time after last success return from readMeasurement
  Serial.println();
}

struct Detector_t {
  int idx;
  int dwellLength;
  int dwellCount;
  float dwellAggregate;
  float lastAverage;
  float stableAverage;

  int tentativeDetection;
  int antiDetection;
  bool detectionInBlock;
  int samplesSinceDetection;

  Detector_t()
  : idx(0),
    dwellLength(22), // ~2 second blocks
    dwellCount(0),
    dwellAggregate(0.F),
    lastAverage(-1.F),
    stableAverage(-1.F),
    tentativeDetection(0),
    antiDetection(0),
    detectionInBlock(false)
  { }
};

#define DETECTOR_VARIANCE_THRESHOLD 4
Detector_t DetectorStatus;

void stepDetector() {
  // Algorithm
  // - non-overlapt "integration" by averaging a block of samples
  // - if we get a spike above or below the previous average then probably a detection
  // - alternative - augment using differential instead

  // DEBUG("%d %d\n\r", DetectorStatus.idx, (int)DetectorStatus.dwellAggregate);

  float m = MlxStatus.magnitude;
  DetectorStatus.idx ++;
  DetectorStatus.dwellAggregate += m;

  if (DetectorStatus.idx % DetectorStatus.dwellLength == 0) {
    DetectorStatus.dwellCount ++;
    if (DetectorStatus.dwellCount < 5) {
      // Compute a longer average when booted, to stabilise - aggregate over first 5 dwells of ~22 samples (~10 seconds)
      return;
    } else {
      float average = DetectorStatus.dwellAggregate / DetectorStatus.idx;
      DetectorStatus.lastAverage = average;
      DetectorStatus.dwellAggregate = 0;
      DetectorStatus.idx = 0;
      if (DetectorStatus.stableAverage < 0) {
        DetectorStatus.stableAverage = average;
        Serial.print(F("stable-average,"));
        Serial.print(DetectorStatus.stableAverage);
        Serial.println();
        return;
      }
      // Update average if there has been no detection in this block
      if (!DetectorStatus.detectionInBlock) {
        DetectorStatus.stableAverage = average;
        Serial.print(F("stable-average,"));
        Serial.print(DetectorStatus.stableAverage);
        Serial.println();
      }
    }
  }
  if (DetectorStatus.stableAverage < 0) { return; }
  // compare sample against the stable average
  float variance = fabs(m - DetectorStatus.stableAverage);
  if (variance >= DETECTOR_VARIANCE_THRESHOLD) {
    // tentative detection
    DetectorStatus.tentativeDetection ++;
    if (DetectorStatus.tentativeDetection == 2) {
      // detection...
      // go until a double 0
      Serial.print(F("Detection-confirmed,"));
      Serial.print(DetectorStatus.stableAverage);
      Serial.print(',');
      Serial.print(m);
      Serial.println();
    }
    if (DetectorStatus.tentativeDetection > 1) {
      DetectorStatus.antiDetection = 0;
      DetectorStatus.detectionInBlock = true;
    }
  } else {
    DetectorStatus.antiDetection ++;
    
    // Debouncing
    // We allow ....101 as a detection but not ...100
    if (DetectorStatus.tentativeDetection == 1 && DetectorStatus.antiDetection > 1) {
      Serial.println(F("False-alarm"));
    }
    if (DetectorStatus.tentativeDetection > 1 && DetectorStatus.antiDetection > 1) {
      Serial.println(F("Detection-completed"));
      DetectorStatus.tentativeDetection = 0;
      DetectorStatus.antiDetection = 0;
      DetectorStatus.samplesSinceDetection = 0;
    }
  }
  if (DetectorStatus.detectionInBlock && DetectorStatus.tentativeDetection > 0) {
    Serial.print(F("Det:")); Serial.println(m);
  }
  if (DetectorStatus.detectionInBlock) {
    DetectorStatus.samplesSinceDetection ++;
  }
  if (DetectorStatus.samplesSinceDetection > DetectorStatus.dwellLength * 5) {
    Serial.println(F("Detection-cleared"));
    DetectorStatus.detectionInBlock = false; // allow stable average to update again after 5 more dwells
    DetectorStatus.samplesSinceDetection = 0;
  }

  // TODO: if detection extends for too long, then perhaps the average has changed...
}

bool debugRadioTransmitPending = false;
elapsedMillis lastErrorBeacon;
elapsedMillis lastHeartbeat;

void reportFault() {
  lastErrorBeacon = 0;
  Serial.print(F("Fault"));
  Serial.print(','); Serial.print(MlxStatus.lastNopCode, HEX);
  Serial.print(','); Serial.print(MlxStatus.mlxRequestError, HEX);
  Serial.print(','); Serial.print(MlxStatus.mlxReadError, HEX);
  Serial.println();
  if (RadioStatus.sx1276Valid) {
    transmitErrorBeacon();
  }
  static int errorBeaconCount = 0;
  if (errorBeaconCount++ > 5) {
    Serial.println(F("MLX fault - will reset"));
#if defined(XMC_BOARD)
    NVIC_SystemReset();
#endif
    delay(2000);
  }
}

void loopErrorHandler() {
  // if we got a setup fault, sleep a bit then reset
  if (!MlxStatus.mlxValid) {
    Serial.println(F("MLX init fault - will reset shortly"));
  }
  if (!MlxStatus.mlxValid || MlxStatus.mlxReadError || MlxStatus.mlxRequestError) {
    if (lastErrorBeacon > ERROR_BEACON_INTERVAL) {
      reportFault();
    } else {
    }
    // Looks like we always need a delay in the loop if we dont print anything, or things end up hanging...
    delay(500);
    return;
  }
}

void transmitHeartbeat() {
  static long counter = 0;
  byte packet[32];
  const long now = uptime / 1000;
  const long tEvent = MlxStatus.lastMeasureValid / 10;
  const uint16_t m = uint16_t(MlxStatus.magnitude);
  const int t = MlxStatus.values.t;
  int tc = 0;
#if defined(XMC_BOARD)
    tc = devCtrl.getTemperature();
#endif
  byte n = 0;
  packet[n++] = 12;
  packet[n++] = 0x5f;  // not really sF, but hey
  packet[n++] = 0xbb;     // this type of message
  packet[n++] = (counter >> 8) & 0xff; // auto wrap counter @ 255 packets, just useful for detecting skip
  packet[n++] = (counter & 0xff);
  // uptime
  packet[n++] = now & 0xff;
  packet[n++] = (now >> 8) & 0xff;
  packet[n++] = (now >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  // last valid measurement
  packet[n++] = tEvent & 0xff;
  packet[n++] = (tEvent >> 8) & 0xff;
  packet[n++] = (tEvent >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  // last measurement
  packet[n++] = m & 0xff;
  packet[n++] = (m >> 8) & 0xff;
  packet[n++] = t;
  packet[n++] = tc;
  packet[n++] = int(DetectorStatus.stableAverage) & 0xff;
  packet[n++] = (int(DetectorStatus.stableAverage) >> 8) & 0xff;
  // TODO time of last detection start/end packet[n++] = t2;
  packet[n++] = 0;
  packet[n++] = 0;
  // DEBUG("transmitDebugCollectionFrame %d\n\r", counter);
  transmitPacket(packet, n);
  counter ++;
}

bool heartbeat() {
  if (lastHeartbeat >= HEARTBEAT_BEACON_MS) {
    digitalWrite(LED_BUILTIN, HIGH);
    int tc = 0;
#if defined(XMC_BOARD)
    tc = devCtrl.getTemperature();
#endif
    lastHeartbeat = 0;
    Serial.print("Heartbeat,");
    Serial.print(DetectorStatus.stableAverage);
    Serial.print(',');
    Serial.print(uptime);
    Serial.print(',');
    Serial.print(MlxStatus.values.t);
    Serial.print(',');
    Serial.print(tc);
    Serial.print(',');
    Serial.print(vibration);
    // todo: stats
    Serial.println();
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    transmitHeartbeat();
    return true;
  }
  return false;
}

void loop() {
  loopErrorHandler();

  // We need to interleave requesting and reading the MLX results with transmitting the previous result
  // because both have a time to conclusion
  // DEBUG("loop %ld\n\r", (long)uptime);

  // This will return true if a measurement was requested (or an error occurred) - which it will be if none is pending
  // the question is, should there be a minimum interval between reading and next requests (so far, it appears not)
  bool priorRequestError = MlxStatus.mlxRequestError;
  measureMlxRequestIf();

  // This will transmit the latest state, at full bore, as long as last request (not reading) succeeded
  // measureValid will be false if request succeeded then reading failed
  bool sentDebugRadioPacket = false;
  if (modeDebugRadioAllMeasurements && MlxStatus.measureValid && debugRadioTransmitPending) {
    if (RadioStatus.sx1276Valid) {
      transmitDebugCollectionFrame();
    } else {
      delay(Radio.PredictTimeOnAir(13) + 8);
    }
  } else {
    // prevent overheating due to tight loop
    // hypothesis - we end up up-rounding the mlx measure interval to tDelay x k
    //delay(500);
    sentDebugRadioPacket = true;
  }
  debugRadioTransmitPending = false;

  // This will return true whether reading completed or an error occurred
  bool priorReadError = MlxStatus.mlxReadError;
  if (measureMlxIf()) {
    MlxStatus.allFrameCounter ++;

    stepDetector();

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
  if (newFault) {
    reportFault();
    return;
  }
  if (!heartbeat()) {
    if (!sentDebugRadioPacket) {
      delay(15);
    }
  }
}
