#include <Arduino.h>
#include <MLX90393.h>
#include <elapsedMillis.h>
#include <SPI.h>
#include <Wire.h>
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

#define VERBOSE 0

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

#define LED_MAIN    LED_BUILTIN   // Arduino 14
#define LED_XTRA    LED2          // Arduino 15

#define PIN_SX1276_RST  4  // hw 5
#define PIN_SX1276_CS   3  // hw 4
#define PIN_SX1276_MISO 0  // hw 1
#define PIN_SX1276_MOSI 1  // hw 2
#define PIN_SX1276_SCK  2  // hw 3

#define PIN_SDA 11         // hw 11
#define PIN_SCL 10         // hw 12

#define PIN_VIBRATION   9
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

#elif defined(TEENSYDUINO)

#define PIN_VIBRATION   5 // at the moment this is the pin used for DHT which we dont need
#define LED_MAIN LED_BUILTIN
#define PIN_SX1276_RST  21
#define PIN_SX1276_MISO PIN_SPI_MISO // 12
#define PIN_SX1276_MOSI PIN_SPI_MOSI // 11
#define PIN_SX1276_CS   22 // alt pin (PIN_SPI_SS == 10)
#define PIN_SX1276_SCK  14 // alt pin (PIN_SPI_SCK == 13, == LED_BUILTIN)

#define PIN_SDA PIN_WIRE_SDA // 18
#define PIN_SCL PIN_WIRE_SCL // 19

#define MLX_IRQ 9

#define ICACHE_FLASH_ATTR

#define BOARD_NAME "TEENSYLC"

#include <InternalTemperature.h>

#else
#error "Unsupported configuration"
#endif

#define SX1276_RESET_LOW_TIME 10
#define SX1276_RESET_WAIT_TIME 50

#define SERIAL_BOOT_DELAY_MS 3500
#define SHORT_LED_FLASH_MS 150

#define ERROR_BEACON_INTERVAL 1000
#define ERROR_BEACON_RESET_AFTER 5

#define DETECTOR_VARIANCE_THRESHOLD 4

#define MEASURE_FLAGS (MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG)

// Set this to true to emulate driveway1, and transmit all frames on radio, in packet compatible with driveway1
bool modeDebugRadioAllMeasurements = false;

// Set this to true, to print all frames to serial
bool modeDebugSerialAllMeasurements = false;

// Set this to 0, to print all frames, otherwise, 10 to print every 10th, to emulate driveway1, etc.
int debugSerialSampleFrameInterval = 0; //10;

#define HEARTBEAT_BEACON_MS 15000

elapsedMillis uptime;

volatile byte vibration = 0;

bool debugRadioTransmitPending = false;
elapsedMillis lastErrorBeacon;
elapsedMillis lastHeartbeat;

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

struct Detector_t {
  int idx;
  uint16_t dwellLength;
  uint16_t dwellCount;
  float dwellAggregate;
  float recentAverage;
  float stableAverage;

  uint16_t tentativeDetection;
  uint16_t antiDetection;
  uint16_t samplesSinceDetection;
  uint16_t continuousDetectingDwells; // used to detect "permanent" background change

  uint16_t id;
  long lastDetectionStart;
  uint16_t lastDetectionDuration;
  float variationIntegral;

  bool detectionInBlock;

  Detector_t()
  : idx(0),
    dwellLength(22), // ~2 second blocks
    dwellCount(0),
    dwellAggregate(0.F),
    recentAverage(-1.F),
    stableAverage(-1.F),
    tentativeDetection(0),
    antiDetection(0),
    continuousDetectingDwells(0),
    id(0),
    lastDetectionStart(0),
    lastDetectionDuration(0),
    variationIntegral(0.F),
    detectionInBlock(false)
  { }
};

RadioStatus_t RadioStatus;
MlxStatus_t MlxStatus;
Detector_t DetectorStatus;

template<typename T> T getBoardTemperature() {
#if defined(XMC_BOARD)
  return devCtrl.getTemperature();
#elif defined(TEENSYDUINO)
  return InternalTemperature.readTemperatureC();
#else
  return 0;
#endif
}

// For the teensy, we just flash the one LED on a heartbeat only
// Perhaps it turns out, on the XMC the LED were part of the problem?

void led_mlx(uint8_t val) {
#if !defined(TEENSYDUINO)
  digitalWrite(LED_MAIN, val);
#endif
}

void led_tx(uint8_t val) {
#if !defined(TEENSYDUINO)
  digitalWrite(LED_XTRA, val);
#endif
}

void led_short_flash(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delay(SHORT_LED_FLASH_MS);
  digitalWrite(pin, LOW);
}

// Used at end of boot when serial port is ready, and a second time after first background reading
void led_five_short_flash(uint8_t pin) {
  for (int n=0; n < 5; n++) {
    delay(SHORT_LED_FLASH_MS);
    led_short_flash(pin);
  }
}

void setup_led() {
  pinMode(LED_MAIN, OUTPUT);
#if defined(XMC_BOARD)
  // XMC has two LED so we can split them up
  pinMode(LED_XTRA, OUTPUT);
#endif
}

void welcome() {
  digitalWrite(LED_MAIN, HIGH);
#if defined(XMC_BOARD)
  digitalWrite(LED_XTRA, HIGH);
#endif
  delay(SERIAL_BOOT_DELAY_MS);
  digitalWrite(LED_MAIN, LOW);
#if defined(XMC_BOARD)
  digitalWrite(LED_XTRA, LOW);
#endif

  led_five_short_flash(LED_MAIN);

  Serial.begin(115200);
  Serial.println(F("SentriFarm Magnetic Field Disruption Probe"));
  Serial.print(F("Device: "));
  Serial.println(F(BOARD_NAME));
}

void lets_get_started() {
  led_five_short_flash(LED_MAIN);
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
  Wire.begin();
  for (byte addr=3; addr < 127; addr++) {
    char buf[32];
    snprintf(buf, sizeof(buf), "Scanning i2c addr %02x", (int)addr);      Serial.println(buf);
    Wire.beginTransmission(addr);
    int error = Wire.endTransmission();
    if (error == 0) {
      snprintf(buf, sizeof(buf), "i2c device at %02x", (int)addr);
      Serial.println(buf);
    }
  }
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
  Serial.print(F(" mrq=")); Serial.print(mrq);
  Serial.println();
}

void vibrationSensorInterruptHandler() {
  vibration++;
}

void setup() {
  setup_led();
  welcome();

  pinMode(PIN_VIBRATION, INPUT_PULLDOWN);
  attachInterrupt(PIN_VIBRATION, vibrationSensorInterruptHandler, FALLING);

  setup_radio();
  print_radio_state();

  setup_mlx();
  print_mlx_state();

  lets_get_started();
}

bool measureMlxRequestIf() {
  if (MlxStatus.measurePending) {
    return false;
  }
  // Note, we dont have any timing constraints on how soon after a reading we can request again...
  // DEBUG("measureMlxRequestIf %ld %d %d\n\r", (long)MlxStatus.lastRequest, digitalRead(MLX_IRQ), getBoardTemperature());
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
    led_mlx(HIGH);
    // DEBUG("measureMlxIf %ld %ld %u %d\n\r", (long)MlxStatus.lastRequest, (long)uptime, MlxStatus.minDelayHeuristic, digitalRead(MLX_IRQ));
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
    led_mlx(LOW);
    return true;
  }
  return false;
}

bool transmitPacket(const void *payload, byte len) {
  bool ok = false;
  led_tx(HIGH);
  SPI.begin();
  if (!Radio.TransmitMessage(payload, len, false)) {
    // TX TIMEOUT - interrupt bit not set by the predicted toa...
    Radio.Standby(); // in case...
  } else {
    // Radio.Standby on success should not be required...
    ok = true;
  }
  SPI.end();
  led_tx(LOW);
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
  byte packet[16];
  const long tEvent = MlxStatus.lastMeasureValid / 10;
  const uint16_t m = uint16_t(MlxStatus.magnitude);
  const int t = MlxStatus.values.t;
  byte n = 0;
  packet[n++] = 12;
  packet[n++] = 0x5f;  // not really sF, but hey
  packet[n++] = 1;     // this type of message
  packet[n++] = (counter >> 8) & 0xff; // auto wrap counter @ 65535 packets, just useful for detecting skip
  packet[n++] = (counter & 0xff);
  packet[n++] = tEvent & 0xff;
  packet[n++] = (tEvent >> 8) & 0xff;
  packet[n++] = (tEvent >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  packet[n++] = m & 0xff;
  packet[n++] = (m >> 8) & 0xff;
  // Pack temp into 6 bits. We cant handle negative for now, but ths is for development anyway
  packet[n++] = (t & 0x3f) | ((MlxStatus.resetCount & 0x3) << 6);
  packet[n++] = 0;
  packet[0] = n;
  packet[n++] = 0;
  // DEBUG("transmitDebugCollectionFrame %d\n\r", counter);
  transmitPacket(packet, n);
  counter ++;
}

void transmitDetection() {
  static long counter = 0;
  byte packet[17];
  byte n = 0;
  uint32_t tEvent = DetectorStatus.lastDetectionStart / 10;
  uint16_t duration = DetectorStatus.lastDetectionDuration / 100;
  uint16_t var = min(65535, DetectorStatus.variationIntegral);
  uint16_t stable = DetectorStatus.stableAverage;
  uint16_t id = DetectorStatus.id;
  packet[n++] = 12;
  packet[n++] = 0x5f;  // not really sF, but hey
  packet[n++] = 2;     // this type of message
  packet[n++] = (counter >> 8) & 0xff; // auto wrap counter @ 65535 packets, just useful for detecting skip
  packet[n++] = (counter & 0xff);
  packet[n++] = tEvent & 0xff;
  packet[n++] = (tEvent >> 8) & 0xff;
  packet[n++] = (tEvent >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  packet[n++] = (id >> 8) & 0xff;
  packet[n++] = id & 0xff;
  packet[n++] = (duration >> 8) & 0xff;
  packet[n++] = duration & 0xff;
  packet[n++] = (stable >> 8) & 0xff;
  packet[n++] = stable & 0xff;
  packet[n++] = (var >> 8) & 0xff;
  packet[n++] = var & 0xff;
  packet[0] = n;
  packet[n++] = 0;
  transmitPacket(packet, n);
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

bool stepDetector() {
  // Algorithm
  // - non-overlapt "integration" by averaging a block of samples
  // - if we get a spike above or below the previous average then probably a detection
  // - alternative - augment using differential instead

  // DEBUG("%d %d\n\r", DetectorStatus.idx, (int)DetectorStatus.dwellAggregate);
  bool transmitted = false;
  float m = MlxStatus.magnitude;
  DetectorStatus.idx ++;
  DetectorStatus.dwellAggregate += m;

  if (DetectorStatus.idx % DetectorStatus.dwellLength == 0) {
    DetectorStatus.dwellCount ++;
    if (DetectorStatus.dwellCount < 5) {
      // Compute a longer average when booted, to stabilise - aggregate over first 5 dwells of ~22 samples (~10 seconds)
      return false;
    } else {
      float average = DetectorStatus.dwellAggregate / DetectorStatus.idx;
      DetectorStatus.recentAverage = average;
      DetectorStatus.dwellAggregate = 0;
      DetectorStatus.idx = 0;
      if (DetectorStatus.stableAverage < 0) {
        DetectorStatus.stableAverage = average;
        Serial.print(F("stable-average,"));
        Serial.print(DetectorStatus.stableAverage);
        Serial.println();
        return false;
      }
      // Update average if there has been no detection in this block
      if (!DetectorStatus.detectionInBlock) {
        DetectorStatus.stableAverage = average;
        Serial.print(F("stable-average,"));
        Serial.print(DetectorStatus.stableAverage);
        Serial.println();
      } else {
        DetectorStatus.continuousDetectingDwells ++;
        if (DetectorStatus.continuousDetectingDwells > 20) {
          // est. 3 minutes...
          Serial.print(F("Extended detection period... update stable background"));
          DetectorStatus.stableAverage = average;
          Serial.print(F("new-stable-average,"));
          Serial.print(DetectorStatus.stableAverage);
          Serial.println();
          DetectorStatus.continuousDetectingDwells = 0;
        }
      }
    }
  }
  if (DetectorStatus.stableAverage < 0) { return false; }
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
      DetectorStatus.id ++;
      DetectorStatus.lastDetectionStart = uptime;
      DetectorStatus.continuousDetectingDwells ++;
      DetectorStatus.variationIntegral = 0;
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
      DetectorStatus.tentativeDetection = 0;
      DetectorStatus.antiDetection = 0;
    }
    else if (DetectorStatus.tentativeDetection > 1 && DetectorStatus.antiDetection > 1) {
      Serial.println(F("Detection-completed"));
      DetectorStatus.lastDetectionDuration = uptime - DetectorStatus.lastDetectionStart;
      DetectorStatus.tentativeDetection = 0;
      DetectorStatus.antiDetection = 0;
      DetectorStatus.samplesSinceDetection = 0;
    }
  }
  if (DetectorStatus.detectionInBlock && DetectorStatus.tentativeDetection > 0) {
    DetectorStatus.variationIntegral += m;
    Serial.print(F("Det:")); Serial.print(m);
    Serial.print(F(", Int:")); Serial.println(DetectorStatus.variationIntegral);
    // schedule this outside... transmitDetection();
    transmitted = true; // indicate to loop to not call delay()
  }
  if (DetectorStatus.detectionInBlock) {
    DetectorStatus.samplesSinceDetection ++;
  }
  if (DetectorStatus.samplesSinceDetection > DetectorStatus.dwellLength * 5) {
    Serial.println(F("Detection-cleared"));
    DetectorStatus.detectionInBlock = false; // allow stable average to update again after 5 more dwells
    DetectorStatus.samplesSinceDetection = 0;
    DetectorStatus.tentativeDetection = 0;
    transmitted = false;
  }
  return transmitted;
}

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
#elif defined(TEENSYDUINO)
    tc = InternalTemperature.readTemperatureC();
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
  packet[0] = n;
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
#elif defined(TEENSYDUINO)
    tc = InternalTemperature.readTemperatureC();
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

  bool transmitted = false;

  static byte lastVibration = 0;
  byte v = vibration;
  if (v != lastVibration) {
    lastVibration = v;
    Serial.print(F("Vibration...")); Serial.println(v);
    // TODO - use a latching heuristic, send a message...
  }
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

    //elapsedMillis t0;
    transmitted = stepDetector();
    if (transmitted) {
      transmitDetection(); // TODO - work out why this is slowing the loop
    }
    //Serial.println(t0);

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
  if (!transmitted && !heartbeat()) {
    if (!sentDebugRadioPacket) {
      if (MlxStatus.lastRequest + 15 < MlxStatus.minDelayHeuristic) {
        delay(15);
      }
    }
  }
}
