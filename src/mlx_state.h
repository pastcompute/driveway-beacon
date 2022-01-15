#ifndef DRIVEWAY_MONITOR_MLX_STATE_H__
#define DRIVEWAY_MONITOR_MLX_STATE_H__

MLX90393 MlxSensor;

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

MlxStatus_t MlxStatus;

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

#endif
