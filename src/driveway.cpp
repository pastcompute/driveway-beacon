#include <Arduino.h>
#include <MLX90393.h>
#include <elapsedMillis.h>
#include <SPI.h>

#include "boards.h"
#include "debug.h"
#include "radio_state.h"
#include "mlx_state.h"
#include "detector.h"

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

driveway::Detector Detector;

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

  Detector.setThreshold(DETECTOR_VARIANCE_THRESHOLD);
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
  const int stable = Detector.getStableAverage();
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
  packet[n++] = stable & 0xff;
  packet[n++] = (stable >> 8) & 0xff;
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
    Serial.print(Detector.getStableAverage());
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
    transmitted = Detector.next(MlxStatus.magnitude);
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
