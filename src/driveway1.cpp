/*
  Copyright (c) 2021 Andrew McDonnell <bugs@andrewmcdonnell.net>

  This file is part of the SentriFarm Driveway Monitor

  SentriFarm Driveway Monitor is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  SentriFarm Radio Relay is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with SentriFarm Driveway Monitor.  If not, see <http://www.gnu.org/licenses/>.
*/

// Enable pin 6/7 TX/RX instead of USB
// From pio, use -DSERIAL_ONBOARD=1

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

// Design decisions:
// - intentionally use a single ino and hybrid C/C++ instead of a modularised series of h/cpp files for main code
//    - reduces maintenance overhead and aides relative simplicity at cost of a longer file
// - simplify error handling
//    - fall to a failed state until next power cycle
//    - if not the radio, just intermittently broadcast
//    - if the radio, all we can do is flash a LED (which we wont see) and/or intermittently try and buzz something

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

#define SERIAL_BOOT_DELAY_MS 2500
#define SHORT_FLASH_MS 150

#define SX1276_RESET_LOW_TIME 10
#define SX1276_RESET_WAIT_TIME 50

#define RADIO_CARRIER_HZ 920000000

#define BACKGROUND_THRESHOLD_COUNT 36

#define STATE_ERROR 0
#define STATE_BOOT 1
#define STATE_ESTIMATE_BACKGROUND 2
#define STATE_NORMAL 3

// Error codes for when the radio still works
#define ERROR_MLX_SENSOR_FAILED_TO_START 2
#define ERROR_MLX_SENSOR_ERROR_AFTER_START 3
#define ERROR_MLX_VALUE_OUT_OF_RANGE 4
#define ERROR_MLX_SENSOR_ERROR_NEXT_START 5
#define ERROR_MLX_SENSOR_ERROR_NEXT_READ 6

// Track an average background magnetic field strength.
// These values are saved in the same units as returned by convertRaw(), but rounded from float
struct BackgroundInfo_t {
  bool valid;
  bool stable;
  int movingSum;
  int history[10];
  int count;
  int movingAverage;
  BackgroundInfo_t() : valid(false), stable(false), movingSum(0), count(0) {}
};

static struct SystemStatus_t {
  bool radioFault;
  bool mlxFault;

  byte radioVersion;
  uint16_t mlxMinDelayHeuristic; // minDelay, rounded up to 1 if < 1000 uS, otherwise, +10ms
  
  byte systemState;
  byte oldState;
  byte lastErrorCode;

  elapsedMillis tLastFaultFlash;

  BackgroundInfo_t backgroundInfo;
  BackgroundInfo_t baselineBackground;

  bool pendingSensorReading;
  elapsedMillis tSensorReadingRequested;

  elapsedMillis tStart;
  uint32_t lastTime;
  int lastMagnitude;
  int lastTemperatureC;

  // for the moment, just push all data so we can do post processing on the PC to
  // work out a more efficient edge processing & algorithm
  bool pendingTransmission1;

  SystemStatus_t() :
    mlxFault(true),
    radioFault(true),
    radioVersion(0),
    mlxMinDelayHeuristic(0),
    systemState(STATE_BOOT),
    oldState(-1),
    lastErrorCode(0),
    pendingSensorReading(false),
    lastTime(0),
    lastMagnitude(0),
    lastTemperatureC(-99),
    pendingTransmission1(false)
  {}
} SystemStatus;

static SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
static SX1276Radio Radio(PIN_SX1276_CS, spiSettings, true);
static MLX90393 MlxSensor;

// Care - not re-entrant
static char* byte2hex(byte b) {
  static char buf[3] = { 0,0,0 };
  char c;
  c = (b >> 4) ; buf[0] = c > 9 ? (c + ('a' - 10)) : (c + '0');
  c = (b & 0xf); buf[1] = c > 9 ? (c + ('a' - 10)) : (c + '0');
  return buf;
}

static void led_clear() {
  digitalWrite(PIN_LED_MAIN, LOW);
  digitalWrite(PIN_LED_XTRA, LOW);
}

static void led_short_flash(uint8_t pin = PIN_LED_MAIN) {
  digitalWrite(pin, HIGH);
  delay(SHORT_FLASH_MS);
  digitalWrite(pin, LOW);
}

// Used at end of boot when serial port is ready, and a second time after first background reading
static void led_five_short_flash() {
  delay(SHORT_FLASH_MS);
  led_short_flash();
  delay(SHORT_FLASH_MS);
  led_short_flash();
  delay(SHORT_FLASH_MS);
  led_short_flash();
  delay(SHORT_FLASH_MS);
  led_short_flash();
  delay(SHORT_FLASH_MS);
  led_short_flash();
}

static void reset_radio() {
  digitalWrite(PIN_SX1276_RST, LOW);
  delay(SX1276_RESET_LOW_TIME); // spec states to pull low 100us then wait at least 5 ms
  digitalWrite(PIN_SX1276_RST, HIGH);
  delay(SX1276_RESET_WAIT_TIME);
}

static void setup_radio() {
  SPI.begin();
  // Just read this once, rather than continuously
  SystemStatus.radioVersion = Radio.ReadVersion();
  if (!Radio.Begin()) {
    Serial.println(F("Init Fault: SX1276"));
  } else {
    // - Default spreading factor is 9 - reducing to 7 significantly reduces ToA
    Radio.SetSpreadingFactor(7);
    Radio.SetCarrier(920000000);
    // - Increasing BW reduces ToA but theoretically reduces range
    Radio.SetBandwidth(SX1276_LORA_BW_125000);

    // For 13 bytes: ToA, Tx, revisit
    // 125000,8 --> 91 99 135
    // 125000,7 --> 50 ? ?
    // 62500,8 --> ? ? 226
    // 62500,7 --> 102 111 148
    // Problem is loss of comms at over a few cm of range - due to antenna, or something else? or bug in rx?

    SystemStatus.radioFault = false;
  }
  SPI.end();
}

static void setup_mlx() {
  if (MLX90393::STATUS_OK != MlxSensor.begin(0, 0)) {
    // try a nop to get the actual status byte
    auto v = MlxSensor.nop();
    Serial.print(F("Init Fault: MLX90393 "));
    Serial.println(byte2hex(v));
    SystemStatus.mlxFault = false;
  } else {
    MlxSensor.reset(); // beware, this changes defaults from begin()
    MlxSensor.setGainSel(7);
    MlxSensor.setResolution(0, 0, 0);
    MlxSensor.setOverSampling(3); // increases mindelay
    MlxSensor.setTemperatureCompensation(0);
    MlxSensor.setDigitalFiltering(5); // reduces mindelay
    auto minDelay = MlxSensor.convDelayMillis();
    SystemStatus.mlxMinDelayHeuristic = (minDelay < 1) ? 1 : minDelay + 10;
    SystemStatus.mlxFault = false;
  }
}

static void print_radio_state() {
  // Read settings directly out of the radio as a sanity check
  uint32_t carrier_hz = 0;
  byte sf = 0;
  SPI.begin();
  Radio.ReadCarrier(carrier_hz);
  Radio.ReadSpreadingFactor(sf);
  SPI.end();

  Serial.print(F("SX1276: ver=")); Serial.print(SystemStatus.radioVersion);
  Serial.print(F(" fault=")); Serial.print(SystemStatus.radioFault);
  Serial.print(F(" tune=")); Serial.print(carrier_hz);
  Serial.print(F(" sf=")); Serial.print(sf);
  Serial.print(F(" toa(13 bytes)=")); Serial.print(Radio.PredictTimeOnAir(13));
  Serial.println();
}

static void print_mlx_state() {
  uint8_t gainSel, hall, osr, rx, ry, rz, df;
  MlxSensor.getGainSel(gainSel);
  MlxSensor.getHallConf(hall);
  MlxSensor.getOverSampling(osr);
  MlxSensor.getResolution(rx, ry, rz);
  MlxSensor.getDigitalFiltering(df);
  Serial.print(F("MLX90393: minDelayHeur=")); Serial.print(SystemStatus.mlxMinDelayHeuristic);
  Serial.print(F(" gainSel=")); Serial.print(gainSel);
  Serial.print(F(" hall=")); Serial.print(hall);
  Serial.print(F(" osr=")); Serial.print(osr);
  Serial.print(F(" filter=")); Serial.print(df);
  Serial.println();
}

static void print_sys_state() {
  Serial.print(F("System: state=")); Serial.print(SystemStatus.systemState);
  Serial.print(F(" oldState=")); Serial.print(SystemStatus.oldState);
  Serial.print(F(" bg.stable=")); Serial.print(SystemStatus.backgroundInfo.stable);
  Serial.print(F(" bg.moving=")); Serial.print(SystemStatus.backgroundInfo.movingAverage);
  Serial.print(F(" bg.samples=")); Serial.print(SystemStatus.backgroundInfo.count);
  Serial.print(F(" bg.runSince=")); Serial.print(SystemStatus.tStart);
  Serial.print(F(" lastRead=")); Serial.print(SystemStatus.lastTime);
  Serial.print(F(" lastMag=")); Serial.print(SystemStatus.lastMagnitude);
  Serial.print(F(" lastDegC=")); Serial.print(SystemStatus.lastTemperatureC);
  Serial.print(F(" error=")); Serial.print(SystemStatus.lastErrorCode);
  Serial.print(F(" mlxFault=")); Serial.print(SystemStatus.mlxFault); 
  Serial.println();
}

void setup() {
  pinMode(PIN_LED_MAIN, OUTPUT);
  pinMode(PIN_LED_XTRA, OUTPUT);
  digitalWrite(PIN_LED_MAIN, HIGH);
  digitalWrite(PIN_LED_XTRA, HIGH);

  pinMode(PIN_SX1276_RST,  OUTPUT);
  pinMode(PIN_SX1276_CS,   OUTPUT);
  digitalWrite(PIN_SX1276_MISO, HIGH);
  digitalWrite(PIN_SX1276_MOSI, HIGH);
  digitalWrite(PIN_SX1276_SCK,  HIGH);
  digitalWrite(PIN_SX1276_CS, HIGH);
  digitalWrite(PIN_SX1276_RST, HIGH);

  reset_radio();

  delay(SERIAL_BOOT_DELAY_MS);
  led_clear();
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

static void reset_background_processing() {
  SystemStatus.backgroundInfo = BackgroundInfo_t();
  Serial.println(F("Starting background estimation processing"));
}

// Return true once we have a stable background
static bool next_background_processing() {
  static elapsedMillis tx;
  MLX90393::txyzRaw raw;
  auto status = MlxSensor.startMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG);
  if (status & MLX90393::ERROR_BIT) {
    Serial.println(F("Cannot measure background!"));
    SystemStatus.lastErrorCode = ERROR_MLX_SENSOR_ERROR_AFTER_START;
    SystemStatus.mlxFault = true;
    return false;
  }
  // if (SystemStatus.mlxMinDelay < 1) { delayMicroseconds(600); } else { delay(SystemStatus.mlxMinDelay+10); }
  delay(SystemStatus.mlxMinDelayHeuristic);
  status = MlxSensor.readMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG, raw);
  if (status & MLX90393::ERROR_BIT) {
    Serial.println(F("Cannot read background!"));
    SystemStatus.lastErrorCode = ERROR_MLX_SENSOR_ERROR_AFTER_START;
    SystemStatus.mlxFault = true;
    return false;
  }

  const MLX90393::txyz values = MlxSensor.convertRaw(raw);

  float magnitude = sqrt(values.x * values.x + values.y * values.y + values.z * values.z);
  if (magnitude >= 32767.F) {
    SystemStatus.lastErrorCode = ERROR_MLX_SENSOR_ERROR_AFTER_START;
    SystemStatus.mlxFault = true;
    return false;
  }

  // keep a moving average, once it is "stable" for some version of stable, we are done
  // for now, this just means we took sufficient recent readings
  // Moving average is sum(lastN)/N, so we need to keep older values to subtract back off
  BackgroundInfo_t& b = SystemStatus.backgroundInfo;
  b.movingSum += magnitude;

  const int N = sizeof(b.history)/sizeof(b.history[0]);
  const auto pos = b.count % N;
  if (b.count > N-1) {
    b.movingSum -= b.history[pos];
    b.movingAverage = b.movingSum / N;
    b.valid = true;

    if (tx > 3000) {
      tx = 0;
      Serial.print(F("Background update: "));
      Serial.println(b.movingAverage);
    }
  }
  b.history[pos] = magnitude;

#if 0
  char buf[200];
  snprintf(buf, sizeof(buf), "DEBUG: bg,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    (int)values.x, (int)values.y, (int)values.z, (int)values.t, (int)magnitude,
    N, b.count, b.movingSum, b.movingAverage, pos, b.history[pos]);
  Serial.println(buf);
#endif

  b.count ++;
  return (b.stable = (b.count > BACKGROUND_THRESHOLD_COUNT));
}

static void transmit1_start() {
  // first draft - just do a full synchronous transmit
  byte packet[16];

  static int counter = 0;

  // include time so we have some idea for proper statistical analysis
  uint32_t tEvent = SystemStatus.lastTime / 10;
  packet[0] = 12;
  packet[1] = 0x5f;  // not really sF, but hey
  packet[2] = 1;     // this type of message
  packet[3] = (counter >> 8) & 0xff;
  packet[4] = (counter & 0xff);
  packet[5] = tEvent & 0xff;
  packet[6] = (tEvent >> 8) & 0xff;
  packet[7] = (tEvent >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  packet[8] = SystemStatus.lastMagnitude & 0xff;
  packet[9] = (SystemStatus.lastMagnitude >> 8) & 0xff;
  packet[10] = SystemStatus.lastTemperatureC;
  packet[11] = min(SystemStatus.backgroundInfo.movingAverage, 0xff);
  packet[12] = 0;
#if 0
  byte xxor = 0;
  for (int n=0; n < 12; n++) { xxor ^= packet[n]; }
  packet[12] = xxor;
  packet[13] = 0; // hack for possibly dodgy rx
#endif
  
  digitalWrite(LED2, HIGH);
  SPI.begin();
  if (!Radio.TransmitMessage(packet, 13, false)) {
    // TX TIMEOUT - interrupt bit not set by the predicted toa...
  }
  Radio.Standby();
  SPI.end();

  //led_short_flash(LED2); // <-- we can't do this it wastes 150ms...
  digitalWrite(LED2, LOW);

  counter ++;
}

static void next_processing() {
  // next_background_processing();

  if (!SystemStatus.pendingSensorReading) {
    // Asynchronously start to read the sensor
    auto status = MlxSensor.startMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG);
    if (status & MLX90393::ERROR_BIT) {
      Serial.println(F("Cannot measure now!"));
      SystemStatus.lastErrorCode = ERROR_MLX_SENSOR_ERROR_NEXT_START;
      SystemStatus.mlxFault = true;
      return;
    }
    elapsedMillis started;
    SystemStatus.tSensorReadingRequested = started;
    SystemStatus.pendingSensorReading = true;
  }
  // OK, while we are waiting for the next, send of the previous one...
  elapsedMillis t1;
  elapsedMillis t2;
  if (SystemStatus.pendingTransmission1) {
    // sf7, bw62500, 14bytes == 286ms (after removing xor and LED), way longer than toa of 102?
    transmit1_start();
    t2 = 0;
    SystemStatus.pendingTransmission1 = false;
  }
  if (SystemStatus.pendingSensorReading && SystemStatus.tSensorReadingRequested >= SystemStatus.mlxMinDelayHeuristic) {
    MLX90393::txyzRaw raw;
    auto status = MlxSensor.readMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG, raw);
    if (status & MLX90393::ERROR_BIT) {
      Serial.println(F("Cannot read now!"));
      SystemStatus.lastErrorCode = ERROR_MLX_SENSOR_ERROR_NEXT_READ;
      SystemStatus.mlxFault = true;
      return;
    }
    SystemStatus.pendingSensorReading = false;
    const MLX90393::txyz values = MlxSensor.convertRaw(raw);
    float magnitude = sqrt(values.x * values.x + values.y * values.y + values.z * values.z);
    // TODO: handle wrap
    SystemStatus.lastTime = SystemStatus.tSensorReadingRequested.get() - SystemStatus.tStart.get();
    SystemStatus.lastMagnitude = magnitude;
    SystemStatus.lastTemperatureC = values.t;
    SystemStatus.pendingTransmission1 = true;
#if 1
    // Slow this down so we dont overload the esp01 when attached for debug
    static int counterSer = 0;
    if (counterSer ++ % 10 == 0) {
      Serial.print(counterSer-1); Serial.print(',');
      Serial.print(SystemStatus.lastTime); Serial.print(',');
      Serial.print(SystemStatus.tSensorReadingRequested); Serial.print(',');
      Serial.print((int)magnitude); Serial.print(',');
      Serial.print((int)values.t);
      Serial.println();
    }
#endif
  }
}

int resetCount = 0;
elapsedMillis resetPending = 0;

static void serial_debug_terminal() {
  while (Serial.available()) {
    auto c = (char) Serial.read();
    switch (c) {
      case 'x':
        resetCount = 0;
        print_sys_state();
        print_radio_state();
        print_mlx_state();
        break;
      case 'e':
        // clear errors
        Serial.println(F("Clearing MLX error code..."));
        SystemStatus.systemState = STATE_NORMAL;
        SystemStatus.lastErrorCode = 0;
        SystemStatus.pendingSensorReading = false;
        SystemStatus.pendingTransmission1 = false;
        break;

      case 'r':
        // Reset the device...
        if (resetCount == 0) {
          resetPending = 0;
          Serial.println(F("Reset request pending... press r 4 more times in 10 seconds"));
        }
        resetCount++;
        if (resetCount >= 5) {
          if (resetPending < 10000) {
            Serial.println(F("Reset requested..."));
            // reset here...
          }
        }
        break;
      default:
        resetCount = 0;
        break;
    }
  }
}

void loop() {
  if (resetCount > 0 && resetPending >= 10000) {
    Serial.println(F("Reset request timed out..."));
    resetPending = 0;
    resetCount = 0;
  }

  bool stateChanged = (SystemStatus.oldState != SystemStatus.systemState);
  if (stateChanged) {
    Serial.print(F("STATE CHANGE "));
    Serial.print(SystemStatus.oldState);
    Serial.print(F(" -> "));
    Serial.print(SystemStatus.systemState);
    Serial.println();
  }
  serial_debug_terminal();

  // Processing requirements.
  // - Intermittently, send a heartbeat if idle
  // - Read magnetic field strength as fast as possible
  // - Attempt to establish background.
  // - Perform any edge processing and queue results
  // - Concurrently, send the (possibly reduced) data over the radio in parallel

  // 1. Attempt to establish background
  //
  // KEY REQUIREMENT - WHEN POWERING ON THE MODULE, ENSURE THERE IS NO VEHICLE PARKED ABOVE
  // As a mitigation, we can try and detect an abnormally low fall below background...

  auto curState = SystemStatus.systemState;
  switch (SystemStatus.systemState) {
    case STATE_ERROR:
      if (stateChanged) {
        if (SystemStatus.lastErrorCode == 0 && SystemStatus.mlxFault) {
          SystemStatus.lastErrorCode = ERROR_MLX_SENSOR_FAILED_TO_START;
        }
        Serial.print(F("E: cannot continue due to hard fault! "));
        Serial.println(byte2hex(SystemStatus.lastErrorCode));
      }
      if (SystemStatus.tLastFaultFlash > 1000) {
        SystemStatus.tLastFaultFlash = 0;
        led_short_flash();
        delay(SHORT_FLASH_MS);
        led_short_flash();
      }

      if (!SystemStatus.radioFault) {
        // We can still broadcast error messages...

        // TODO
      }
      break;

    case STATE_NORMAL:
      if (stateChanged && SystemStatus.oldState == STATE_ESTIMATE_BACKGROUND) {
        Serial.print(F("INITIAL ESTIMATED BACKGROUND MOVING AVERAGE: "));
        Serial.println(SystemStatus.backgroundInfo.movingAverage);
        led_five_short_flash();

        SystemStatus.baselineBackground = SystemStatus.backgroundInfo;
        SystemStatus.tStart = 0;
      }
      next_processing();
      break;

    case STATE_ESTIMATE_BACKGROUND:
      if (stateChanged && SystemStatus.oldState != STATE_BOOT) {
        Serial.println(F("Restarted background estimation"));
      }
      if (next_background_processing()) {
        SystemStatus.systemState = STATE_NORMAL;
      }      
      break;

    case STATE_BOOT:
    default:
      if (!SystemStatus.mlxFault) {
        SystemStatus.systemState = STATE_ESTIMATE_BACKGROUND;
        reset_background_processing();
      }
      break;
  }
  if (SystemStatus.mlxFault || SystemStatus.radioFault) {
    SystemStatus.systemState = STATE_ERROR;
  } 
  SystemStatus.oldState = curState;
}