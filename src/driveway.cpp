#include <Arduino.h>
#include <MLX90393.h>
#include <elapsedMillis.h>
#include <SPI.h>

#include "boards.h"
#include "debug.h"
#include "protocol.h"
#include "radio.h"
#include "mlxsensor.h"
#include "vibrationsensor.h"
#include "detector.h"

#include <RunningAverage.h>

#define SERIAL_BOOT_DELAY_MS 3500
#define SHORT_LED_FLASH_MS 150

#define ERROR_BEACON_INTERVAL_MS 1000
#define ERROR_BEACON_RESET_AFTER 5

#define DRIVEWAY_DATA_COLLECTOR_ENABLED 0

#define DETECTOR_VARIANCE_THRESHOLD 4

#define HEARTBEAT_BEACON_MS 15000

// Set this to true to emulate driveway1, and transmit all frames on radio, in packet compatible with driveway1
bool modeDebugRadioAllMeasurements = false;

// Set this to true, to print all frames to serial
bool modeDebugSerialAllMeasurements = false;

// Set this to 0, to print all frames, otherwise, 10 to print every 10th, to emulate driveway1, etc.
int debugSerialSampleFrameInterval = 0; //10;

elapsedMillis uptime;

bool debugRadioTransmitPending = false;
elapsedMillis lastErrorBeacon;
elapsedMillis lastHeartbeat;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
const bool inAir9b = true;
SX1276Radio RadioSX1276(PIN_SX1276_CS, spiSettings, inAir9b);

MLX90393 Mlx90393;

driveway::Board Board1;
driveway::Detector Detector;
driveway::Radio Radio(RadioSX1276);
driveway::MlxSensor MlxSensor(Mlx90393);
driveway::VibrationSensor VibrationSensor(PIN_VIBRATION);
driveway::Protocol<driveway::Board, driveway::MlxSensor, driveway::Detector> Protocol(Board1, MlxSensor, Detector);

RunningAverage mlxMeasurementInterval(30);

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
  Serial.print(F("Board: "));
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
  extern void transmitStartBeacon();
  transmitStartBeacon();
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
  Serial.println(); Serial.flush();
}

void printFaultMessage() {
  Serial.print(F("Fault"));
  Serial.print(','); Serial.print(MlxSensor.getLastNopCode(), HEX);
  Serial.print(','); Serial.print(MlxSensor.getRequestError(), HEX);
  Serial.print(','); Serial.print(MlxSensor.getReadError(), HEX);
  Serial.println(); Serial.flush();
}

void printHeartbeatMessage() {
    Serial.print("Heartbeat");
    Serial.print(','); Serial.print(uptime);
    Serial.print(','); Serial.print(Board1.readVcc());
    Serial.print(','); Serial.print(Board1.readTemperatureC());
    Serial.print(','); Serial.print(MlxSensor.getTemperature());
    Serial.print(','); Serial.print(mlxMeasurementInterval.getAverage());
    Serial.print(','); Serial.print(Detector.getStableAverage());
    Serial.print(','); Serial.print(VibrationSensor.getRawCount());
    Serial.println(); Serial.flush();
}

bool heartbeat() {
  if (lastHeartbeat >= HEARTBEAT_BEACON_MS) {
    lastHeartbeat = 0;
    digitalWrite(LED_MAIN, HIGH);
    printHeartbeatMessage();
    LoraMessage msg = Protocol.heartbeat(VibrationSensor.latched());
    Radio.transmitPacket(msg.getBytes(), msg.getLength());
    digitalWrite(LED_MAIN, LOW);
    return true;
  }
  return false;
}

void transmitStartBeacon() {
  char packet[14];
  // send trailing space as a defensive hack against intermittent bug where last byte not reeived
  snprintf(packet, sizeof(packet), "!BOOT,%02x,%d,%d ", MlxSensor.getLastNopCode(), MlxSensor.getRequestError(), MlxSensor.getReadError());
  Radio.transmitPacket(packet, strlen(packet));
}

void transmitErrorBeacon() {
  char packet[14];
  // send trailing space as a defensive hack against intermittent bug where last byte not reeived
  snprintf(packet, sizeof(packet), "!FAULT,%02x,%d,%d ", MlxSensor.getLastNopCode(), MlxSensor.getRequestError(), MlxSensor.getReadError());
  Radio.transmitPacket(packet, strlen(packet));
}

void reportFault() {
  lastErrorBeacon = 0;
  printFaultMessage();
  if (Radio.isValid()) {
    transmitErrorBeacon();
  }
  static int errorBeaconCount = 0;
  if (errorBeaconCount++ > 5) {
    Serial.println(F("MLX fault: reset"));
    // TODO !
    delay(2000);
  }
}

// given the values of request error & read error before and after calling the async functions
// see if fsult state changed
bool detectFaultLatch(bool priorRequestError, bool priorReadError) {
  bool newFault = false;
  bool rqError = MlxSensor.getRequestError();
  bool rxError = MlxSensor.getReadError();
  if (rqError && priorRequestError != rqError) {
    // we just had a request fail start
    Serial.println(F("MLX request fault"));
    newFault = true;
  }
  if (rxError && priorReadError != rxError) {
    // we just had a read fail start
    Serial.println(F("MLX read fault"));
    newFault = true;
  }
  return newFault;
}

bool developmentTransmit() {
#if DRIVEWAY_DATA_COLLECTOR_ENABLED
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
#else
  return false;
#endif
}

void loopErrorHandler() {
  // if we got a setup fault, sleep a bit then reset
  if (!MlxSensor.isValid()) {
    Serial.println(F("MLX init fault - reset soon"));
    delay(5000);
  }
  if (MlxSensor.isError()) {
    if (lastErrorBeacon > ERROR_BEACON_INTERVAL_MS) {
      reportFault();
    } else {
    }
    // Looks like we always need a delay in the loop if we dont print anything, or things end up hanging...
    delay(500);
    return;
  }
}

void loop() {
  // DEBUG("loop %ld\n\r", (long)uptime);
  loopErrorHandler();

  // Update the vibration latch if any interrupts occured since the last loop
  if (long v = VibrationSensor.poll()) {
    Serial.print(F("vibr,")); Serial.println(v);
  }

  // We need to interleave requesting and reading the MLX results with transmitting the previous result
  // because both have a time to conclusion

  // This will return true if a measurement was requested (or an error occurred) - which it will be if none is pending
  // the question is, should there be a minimum interval between reading and next requests (so far, it appears not)
  bool priorRequestError = MlxSensor.getRequestError();
  MlxSensor.measureAsyncStart();

  bool developmentPacketSent = developmentTransmit();

  // This will return true whether reading completed or an error occurred
  bool priorReadError = MlxSensor.getReadError();
  bool transmitted = false;
  bool detection = false;
  if (MlxSensor.measureAsyncComplete()) {
    if (MlxSensor.getCompletedMeasurements() > 0) { mlxMeasurementInterval.add(MlxSensor.getMeasurementInterval()); }
    detection = Detector.next(MlxSensor.getMagnitude());
#if DRIVEWAY_DATA_COLLECTOR_ENABLED
    // signal next loop that developmentTransmit should transmit
    debugRadioTransmitPending = true;

    // print debug to serial, either continuous, or on a sample if we have a constrained debug link like an ESP01
    if (modeDebugSerialAllMeasurements) {
      if ((debugSerialSampleFrameInterval < 1) || (MlxSensor.getCompletedMeasurements() % debugSerialSampleFrameInterval == 0)) {
        printDebugCollectionFrame();
      }
    }
#endif    
  }

  // During the detection keep sending messages
  // later we could refine it by reducing the period to once per second...
  bool vibrationLatched = VibrationSensor.latched();
  if (detection || vibrationLatched) {
    LoraMessage msg = Protocol.detection(vibrationLatched);
    Radio.transmitPacket(msg.getBytes(), msg.getLength()); // TODO - work out why this is slowing the loop
    transmitted = true;
    // this takes 65 milliseconds, which is getting close to the measurement rate of 79
    // which is why we set transmitted to true, so it wont spin-delay below (in any case there is the 15 ms margin)
  }

  bool newFault = detectFaultLatch(priorRequestError, priorReadError);
  // if we have an error state, transmit an error beacon at 5 second intervals
  if (newFault) {
    reportFault();
    return;
  }

  // yield the CPU if we have time to spare
  if (!transmitted && !heartbeat()) {
    if (!developmentPacketSent) {
      if (MlxSensor.getRequestTime() + 15 < MlxSensor.getMinDelayAdjusted()) {
        delay(15);
      }
    }
  }
}
