#ifndef DRIVEWAY_MONITOR_MLX_SENSOR_H__
#define DRIVEWAY_MONITOR_MLX_SENSOR_H__

extern void led_mlx(uint8_t);
extern elapsedMillis uptime;

#define VERBOSE_DEBUG_SETUPMLX 0

#if VERBOSE_DEBUG_SETUPMLX
#define DEBUG_SETUPMLX(x ...) { char buf[128]; snprintf(buf, sizeof(buf), x); Serial.print(buf); }
#else
#define DEBUG_SETUPMLX(x ...)
#endif

namespace driveway {

class MlxSensor
{
private:

  static const uint8_t MEASURE_FLAGS = (MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG);

  MLX90393 &sensor;

  bool mlxValid;
  byte lastNopCode;
  uint16_t resetCount;

  uint16_t minDelayHeuristic;
  
  elapsedMillis lastRequest;
  elapsedMillis lastResult;
  long returnTime;
  bool measurePending;

  bool measureValid;
  long lastMeasureInterval;
  long lastMeasureValid;

  bool mlxRequestError;
  bool mlxReadError;
  MLX90393::txyz values;
  float magnitude;

  long allCompletedMeasurementCounter = 0;

public:

  void setup();
  void reconfigure();
  void printState();
  void printDebugCsv();

  bool isValid() const { return mlxValid; }
  bool isError() const { return !mlxValid || mlxRequestError || mlxReadError; }

  bool measureAsyncStart();
  bool measureAsyncComplete();

  bool getMeasurementValid() const { return measureValid; }
  bool getRequestError() const { return mlxRequestError; }
  bool getReadError() const { return mlxReadError; }
  long getRequestTime() const { return lastRequest; }
  long getMeasurementTime() const { return lastMeasureValid; }
  long getMeasurementInterval() const { return lastMeasureInterval; }
  byte getLastNopCode() const { return lastNopCode; }
  float getMagnitude() const { return magnitude; }
  float getTemperature() const { return values.t; }
  uint16_t getResetCount() const { return resetCount; }
  uint16_t getMinDelayAdjusted() const { return minDelayHeuristic; }
  long getCompletedMeasurements() const { return allCompletedMeasurementCounter; }
  long getReturnTime() const { return returnTime; }

  MlxSensor(MLX90393 &sensor)
  : sensor(sensor),
    mlxValid(false),
    lastNopCode(0),
    resetCount(0),
    minDelayHeuristic(0),
    returnTime(0),
    measurePending(false),
    measureValid(false),
    lastMeasureInterval(0),
    lastMeasureValid(0),
    mlxRequestError(false),
    mlxReadError(false),
    magnitude(0.F)
  { }
};

void MlxSensor::reconfigure() {
  Serial.println(F("conf_mlx"));
  // FIXME WHEN WE WIRE IT UP pinMode(MLX_IRQ, INPUT_PULLDOWN);
  uint8_t r;
  r = this->sensor.reset(); // beware, this changes defaults from begin()
  DEBUG_SETUPMLX("%02x <-- reset\n\r", r);
  r = this->sensor.setGainSel(7);
  DEBUG_SETUPMLX("%02x <-- setGainSel\n\r", r);
  r = this->sensor.setResolution(0, 0, 0);
  DEBUG_SETUPMLX("%02x <-- setResolution\n\r", r);
  r = this->sensor.setOverSampling(3); // increases mindelay
  DEBUG_SETUPMLX("%02x <-- setOverSampling\n\r", r);
  r = this->sensor.setTemperatureCompensation(0);
  DEBUG_SETUPMLX("%02x <-- setTemperatureCompensation\n\r", r);
  r = this->sensor.setDigitalFiltering(5); // reduces mindelay
  DEBUG_SETUPMLX("%02x <-- setDigitalFiltering\n\r", r);
  r = this->lastNopCode = this->sensor.nop();
  DEBUG_SETUPMLX("%02x <-- nop\n\r", r);
  (void)r;
  auto minDelay = this->sensor.convDelayMillis();
  this->minDelayHeuristic = (minDelay < 1) ? 1 : minDelay + 10;
  this->mlxValid = true;
}

void MlxSensor::setup() {
  Serial.println(F("setup_mlx"));
  Serial.flush();
  this->mlxValid = false;
  // For some reason if we dont scan the bus, some boards wont pick it up
  // There also seems to be some dicey interaction with the serial port, at least at this early point.
  Wire.begin();
  for (byte addr=3; addr < 127; addr++) {
    char buf[32];
    Serial.print('.');
    Wire.beginTransmission(addr);
    int error = Wire.endTransmission();
    if (error == 0) {
      snprintf(buf, sizeof(buf), " i2c found @ %02x", (int)addr);
      Serial.print(buf);
      Serial.flush();
    }
  }
  Serial.println();
  Serial.flush();
  for (int retry=0; retry < 3; retry++) {
    delay(500);
    auto s = this->sensor.begin(0, 0, -1, Wire);
    if (MLX90393::STATUS_OK != s) {
      Serial.print(F("MlxInitCode-OR'd="));
      Serial.println(s);
      this->lastNopCode = this->sensor.nop();
      if (this->sensor.hasError(this->lastNopCode)) {
        Serial.print(F("MlxInitFault-nop="));
        Serial.print(this->lastNopCode);
        Serial.println(F(", retrying..."));
        this->sensor.reset();
      }
    } else {
      this->reconfigure();
      break;
    }
  }
  DEBUG("mlxValid=%d,mlxReqErr=%d,mlxReadErr=%d\n\r", mlxValid, mlxRequestError, mlxReadError);
}

void MlxSensor::printState() {
  uint8_t gainSel, hall, osr, rx, ry, rz, df;
  this->sensor.getGainSel(gainSel);
  this->sensor.getHallConf(hall);
  this->sensor.getOverSampling(osr);
  this->sensor.getResolution(rx, ry, rz);
  this->sensor.getDigitalFiltering(df);
  byte nop = this->sensor.nop();
  auto mrq = digitalRead(MLX_IRQ);

  Serial.print(F("MLX90393:"));
  Serial.print(F(" minDelay'=")); Serial.print(this->minDelayHeuristic); 
  Serial.print(F(" gainSel=")); Serial.print(gainSel);
  Serial.print(F(" hall=")); Serial.print(hall);
  Serial.print(F(" osr=")); Serial.print(osr);
  Serial.print(F(" filter=")); Serial.print(df);
  Serial.print(F(" nop=")); Serial.print(nop);
  Serial.print(F(" mrq=")); Serial.print(mrq);
  Serial.print(F(" err=")); Serial.print(isError()); // Serial.print(','); Serial.print(this->mlxRequestError); Serial.print(','); Serial.print(this->mlxReadError);
  Serial.println();
}

bool MlxSensor::measureAsyncStart() {
  if (this->measurePending) {
    return false;
  }
  // Note, we dont have any timing constraints on how soon after a reading we can request again...
  DEBUG("measureMlxRequestIf %ld %d %d\n\r", (long)this->lastRequest, digitalRead(MLX_IRQ), (int)getBoardTemperature<float>());
  this->returnTime = this->lastRequest;
  this->lastRequest = 0;
  auto status = this->sensor.startMeasurement(MEASURE_FLAGS);
  if (status & MLX90393::ERROR_BIT) {
    this->mlxRequestError = true;
    this->lastNopCode = status;
    Serial.println(F("MLX-req-err"));
  } else {
    this->measurePending = true;
  }
  return true;
}

bool MlxSensor::measureAsyncComplete() {
  if (this->mlxRequestError) {
    return false;
  }
  if (this->measurePending && (this->lastRequest >= this->minDelayHeuristic)) {
    led_mlx(HIGH);
    DEBUG("measureMlxIf %ld %ld %u %d\n\r", (long)this->lastRequest, (long)uptime, this->minDelayHeuristic, digitalRead(MLX_IRQ));
    MLX90393::txyzRaw raw;
    this->measureValid = false;
    auto status = this->sensor.readMeasurement(MEASURE_FLAGS, raw);
    if (status & MLX90393::ERROR_BIT) {
      this->mlxReadError = true;
      this->lastNopCode = status;
      Serial.println(F("MLX-read-err"));
    } else {
      auto prior = this->lastMeasureValid;
      this->lastMeasureValid = uptime;
      this->lastMeasureInterval = this->lastMeasureValid - prior;
      MLX90393::txyz& values = this->values = this->sensor.convertRaw(raw);
      this->magnitude = sqrt(values.x * values.x + values.y * values.y + values.z * values.z);
      this->measureValid = true;
    }
    this->measurePending = false;
    led_mlx(LOW);
    this->allCompletedMeasurementCounter++;
    return true;
  }
  return false;
}

void MlxSensor::printDebugCsv() {
  Serial.print(this->allCompletedMeasurementCounter);
  Serial.print(','); Serial.print(this->mlxValid);
  Serial.print(','); Serial.print(this->mlxRequestError);
  Serial.print(','); Serial.print(this->mlxReadError);
  Serial.print(','); Serial.print(this->lastNopCode);
  Serial.print(','); Serial.print(this->values.x);
  Serial.print(','); Serial.print(this->values.y);
  Serial.print(','); Serial.print(this->values.z);
  Serial.print(','); Serial.print(this->values.t);
  Serial.print(','); Serial.print(this->magnitude);
}

}

#endif
