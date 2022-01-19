#include <Arduino.h>
#include <MLX90393.h>
#include <elapsedMillis.h>
#include <SPI.h>

#include "protocol.h"
#include "boards.h"
#include "debug.h"
#include "radio.h"
#include "mlxsensor.h"
#include "vibrationsensor.h"
#include "detector.h"

#define SERIAL_BOOT_DELAY_MS 3500
#define SHORT_LED_FLASH_MS 150

#define ERROR_BEACON_INTERVAL 1000
#define ERROR_BEACON_RESET_AFTER 5

#define DETECTOR_VARIANCE_THRESHOLD 4

// Set this to true to emulate driveway1, and transmit all frames on radio, in packet compatible with driveway1
bool modeDebugRadioAllMeasurements = false;

// Set this to true, to print all frames to serial
bool modeDebugSerialAllMeasurements = false;

// Set this to 0, to print all frames, otherwise, 10 to print every 10th, to emulate driveway1, etc.
int debugSerialSampleFrameInterval = 0; //10;

#define HEARTBEAT_BEACON_MS 15000

elapsedMillis uptime;

bool debugRadioTransmitPending = false;
elapsedMillis lastErrorBeacon;
elapsedMillis lastHeartbeat;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
const bool inAir9b = true;
SX1276Radio RadioSX1276(PIN_SX1276_CS, spiSettings, inAir9b);

MLX90393 MlxSensor90303;

driveway::Detector Detector;
driveway::Radio Radio(RadioSX1276);
driveway::MlxSensor MlxSensor(MlxSensor90303);
driveway::VibrationSensor VibrationSensor(PIN_VIBRATION);

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

void setup() {
  setup_led();
  welcome();

  VibrationSensor.setup();

  Radio.setup();
  Radio.printState();

  MlxSensor.setup();
  MlxSensor.printState();

  lets_get_started();

  Detector.setThreshold(DETECTOR_VARIANCE_THRESHOLD);
}

void transmitErrorBeacon() {
  char packet[14];
  // send trailing space as a defensive hack against intermittent bug where last byte not reeived
  snprintf(packet, sizeof(packet), "FAULT,%02x,%d,%d ", MlxSensor.getLastNopCode(), MlxSensor.getRequestError(), MlxSensor.getReadError());
  Radio.transmitPacket(packet, strlen(packet));
}

void transmitDebugCollectionFrame() {
  static long counter = 0;
  byte packet[16];
  const long tEvent = MlxSensor.getMeasurementTime() / 10;
  const uint16_t m = uint16_t(MlxSensor.getMagnitude());
  const int t = MlxSensor.getTemperature();
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
  packet[n++] = (t & 0x3f) | ((MlxSensor.getResetCount() & 0x3) << 6);
  packet[n++] = 0;
  packet[0] = n;
  packet[n++] = 0;
  // DEBUG("transmitDebugCollectionFrame %d\n\r", counter);
  Radio.transmitPacket(packet, n);
  counter ++;
}

void transmitDetection() {
  static long counter = 0;
  byte packet[17];
  byte n = 0;
  uint32_t tEvent = Detector.getLastDetectionStart() / 10;
  uint16_t duration = Detector.getLastDetectionDuration() / 100;
  uint16_t var = min(65535, Detector.getDetectionIntegral());
  uint16_t stable = Detector.getStableAverage();
  uint16_t id = Detector.getLastId();
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
  Radio.transmitPacket(packet, n);
  counter ++;
}

void printDebugCollectionFrame() {
  auto now = uptime;
  Serial.print(now);
  Serial.print(F(",mlx,"));
  MlxSensor.printDebugCsv();
  Serial.print(F(",sx1276"));
  Serial.print(','); Serial.print(Radio.isValid());
  Serial.print(F(",timing"));
  Serial.print(','); Serial.print(now - MlxSensor.getRequestTime()); // time of call to requestMeasurement
  Serial.print(','); Serial.print(MlxSensor.getReturnTime());        // interval between successive requests (eff. rate)
  Serial.print(','); Serial.print(MlxSensor.getMeasurementTime());  // time after last success return from readMeasurement
  Serial.println();
}

void reportFault() {
  lastErrorBeacon = 0;
  Serial.print(F("Fault"));
  Serial.print(','); Serial.print(MlxSensor.getLastNopCode(), HEX);
  Serial.print(','); Serial.print(MlxSensor.getRequestError(), HEX);
  Serial.print(','); Serial.print(MlxSensor.getReadError(), HEX);
  Serial.println();
  if (Radio.isValid()) {
    transmitErrorBeacon();
  }
  static int errorBeaconCount = 0;
  if (errorBeaconCount++ > 5) {
    Serial.println(F("MLX fault - will reset"));
    // TODO
    delay(2000);
  }
}

void loopErrorHandler() {
  // if we got a setup fault, sleep a bit then reset
  if (!MlxSensor.isValid()) {
    Serial.println(F("MLX init fault - will reset shortly"));
  }
  if (!MlxSensor.isError()) {
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
  //byte packet[32];
  const long now = uptime / 1000;
  const long tEvent = MlxSensor.getMeasurementTime();
  const float m = MlxSensor.getMagnitude();
  const float t = MlxSensor.getTemperature();
  const int stable = Detector.getStableAverage();
  float tc = 0;
#if defined(TEENSYDUINO)
  tc = InternalTemperature.readTemperatureC();
#endif
  // byte n = 0;
  // packet[n++] = 12;
  // packet[n++] = 0x5f;  // not really sF, but hey
  // packet[n++] = 0xbb;     // this type of message
  // packet[n++] = (counter >> 8) & 0xff; // auto wrap counter @ 255 packets, just useful for detecting skip
  // packet[n++] = (counter & 0xff);
  // // uptime
  // packet[n++] = now & 0xff;
  // packet[n++] = (now >> 8) & 0xff;
  // packet[n++] = (now >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  // // last valid measurement
  // packet[n++] = tEvent & 0xff;
  // packet[n++] = (tEvent >> 8) & 0xff;
  // packet[n++] = (tEvent >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  // // last measurement
  // packet[n++] = m & 0xff;
  // packet[n++] = (m >> 8) & 0xff;
  // packet[n++] = t;
  // packet[n++] = tc;
  // packet[n++] = stable & 0xff;
  // packet[n++] = (stable >> 8) & 0xff;
  // // TODO time of last detection start/end packet[n++] = t2;
  // packet[0] = n;
  // packet[n++] = 0;
  // // DEBUG("transmitDebugCollectionFrame %d\n\r", counter);

  lpp.reset();
  lpp.addUnixTime(1, now);
  lpp.addTemperature(1, tc);

  lpp.addUnixTime(2, tEvent);
  lpp.addTemperature(2, t);
  lpp.addGenericSensor(2, m);
  lpp.addDigitalInput(2, counter);
  lpp.addAnalogInput(2, stable);

  Radio.transmitPacket(lpp.getBuffer(), lpp.getSize());
  counter ++;
}

bool heartbeat() {
  if (lastHeartbeat >= HEARTBEAT_BEACON_MS) {
    digitalWrite(LED_BUILTIN, HIGH);
    int tc = 0;
#if defined(TEENSYDUINO)
    tc = InternalTemperature.readTemperatureC();
#endif
    lastHeartbeat = 0;
    Serial.print("Heartbeat,");
    Serial.print(Detector.getStableAverage());
    Serial.print(',');
    Serial.print(uptime);
    Serial.print(',');
    Serial.print(MlxSensor.getTemperature());
    Serial.print(',');
    Serial.print(tc);
    Serial.print(',');
    Serial.print(VibrationSensor.getRawCount());
    // todo: stats
    Serial.println();
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    transmitHeartbeat();
    return true;
  }
  return false;
}

// given the values of request error & read error before and after calling the async functions
// see if fsult state changed
bool detectFaultLatch(bool priorRequestError, bool priorReadError) {
  bool newFault = false;
  bool rqError = MlxSensor.getRequestError();
  bool rxError = MlxSensor.getReadError();
  if (rqError && priorRequestError != rqError) {
    // we just had a request fail start
    Serial.println(F("MLX request fault detect"));
    newFault = true;
  }
  if (rxError && priorReadError != rxError) {
    // we just had a read fail start
    Serial.println(F("MLX read fault detect"));
    newFault = true;
  }
  return newFault;
}

bool developmentTransmit() {
  // This will transmit the latest state, at full bore, as long as last request (not reading) succeeded
  // measureValid will be false if request succeeded then reading failed
  bool packetSent = false;
  if (modeDebugRadioAllMeasurements && MlxSensor.getMeasurementValid() && debugRadioTransmitPending) {
    if (Radio.isValid()) {
      transmitDebugCollectionFrame();
    } else {
      delay(RadioSX1276.PredictTimeOnAir(13) + 8);
    }
    packetSent = true;
  } else {
    // prevent overheating due to tight loop
    // hypothesis - we end up up-rounding the mlx measure interval to tDelay x k
    //delay(500);
  }
  debugRadioTransmitPending = false;
  return packetSent;
}

void loop() {
  loopErrorHandler();

  bool transmitted = false;

  if (long v = VibrationSensor.poll()) {
   Serial.print(F("Vibration...")); Serial.println(v);
    // TODO - use a latching heuristic, send a message...
  }

  // We need to interleave requesting and reading the MLX results with transmitting the previous result
  // because both have a time to conclusion
  // DEBUG("loop %ld\n\r", (long)uptime);

  // This will return true if a measurement was requested (or an error occurred) - which it will be if none is pending
  // the question is, should there be a minimum interval between reading and next requests (so far, it appears not)
  bool priorRequestError = MlxSensor.getRequestError();
  MlxSensor.measureAsyncStart();

  bool packetSent = developmentTransmit();

  // This will return true whether reading completed or an error occurred
  bool priorReadError = MlxSensor.getReadError();
  if (MlxSensor.measureAsyncComplete()) {
    //elapsedMillis t0;
    transmitted = Detector.next(MlxSensor.getMagnitude());
    if (transmitted) {
      transmitDetection(); // TODO - work out why this is slowing the loop
    }
    //Serial.println(t0);

    // signal next loop that developmentTransmit should transmit
    debugRadioTransmitPending = true;

    // print debug to serial, either continuous, or on a sample if we have a constrained debug link like an ESP01
    if (modeDebugSerialAllMeasurements) {
      if ((debugSerialSampleFrameInterval < 1) || (MlxSensor.getCompletedMeasurements() % debugSerialSampleFrameInterval == 0)) {
        printDebugCollectionFrame();
      }
    }
  }

  bool newFault = detectFaultLatch(priorRequestError, priorReadError);
  // if we have an error state, transmit an error beacon at 5 second intervals
  if (newFault) {
    reportFault();
    return;
  }
  // yield the CPU if we have time
  if (!transmitted && !heartbeat()) {
    if (!packetSent) {
      if (MlxSensor.getRequestTime() + 15 < MlxSensor.getMinDelayAdjusted()) {
        delay(15);
      }
    }
  }
}
