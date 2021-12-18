#include <Arduino.h>
#include <MLX90393.h>
#include <elapsedMillis.h>

#define PIN_MLX_DIRQ 15 // XMC1100 IRQ pin

MLX90393 mlx;

elapsedMillis timeElapsed;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  delay(5000);
  Serial.begin(115200);
  Serial.println(F("Hello, World"));

  // GAIN_SEL => 7 --> assume hall_conf (spinning rate) defaults to 0 as not set by begin() --> 
  //     if res 0 --> 0.196, 0.316 (xy, z) sensitivity
  //     if res 1 --> 0.393, 0.692
  //     if res 2 --> 0.785, 1.265
  //     if res 3 --> 1.570, 2.530
  // Res defaults to 0,0,0
  // Magnetic OverSampling --> 3 --> alters conversion time and noise?
  //  --> conv time E 5,6,9,16,28,52,102,200 for filter E 0..7
  // Filter --> 7
  // Temp Comp == 0 --> we should probably consider enabling this, changes range from 2's comp to unsigned and disables res 2,3
  if (MLX90393::STATUS_OK != mlx.begin(0, 0)) { //}, PIN_MLX_DIRQ)) {
    Serial.println(F("Error starting MLX90393"));
  }
  mlx.setDigitalFiltering(5); // 7 --> 259, 6 --> 132, 5--> 68
  timeElapsed = 0;
}

bool seenStatus0 = false;

void loop() {
  char buf[255];
  uint8_t status;

  auto minDelay = mlx.convDelayMillis();

  status = mlx.nop();
  if ( MLX90393::STATUS_ERROR == status) {
    Serial.println(F("Error reading MLX90393"));
    delay(1000);
    return;
  } else if (!seenStatus0) {
    snprintf(buf, sizeof(buf), "STATUS %02x (%c) minDelay=%d", status, (status & MLX90393::ERROR_BIT ? '!':' '), minDelay);
    Serial.println(buf);
    seenStatus0 = true;
  }

  MLX90393::txyzRaw raw;

  mlx.startMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG);
  delay(minDelay);
  if (minDelay < 1) { delayMicroseconds(600); }
  status = mlx.readMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG, raw);

  const MLX90393::txyz values = mlx.convertRaw(raw);
  //snprintf(buf, sizeof(buf), "STATUS %02x (%c) RAW.xyzt %d,%d,%d,%d DATA.xyzt %d,%d,%d,%d minDelay=%d",
  //  status, (status & MLX90393::ERROR_BIT ? '!':' '),
  //  raw.x, raw.y, raw.t, raw.t, (int)values.x, (int)values.y, (int)values.z, (int)values.t, minDelay);
  snprintf(buf, sizeof(buf), "STATUS %02x (%c) DATA.xyzt %d,%d,%d,%d minDelay=%d since=%lu",
    status, (status & MLX90393::ERROR_BIT ? '!':' '),
    (int)values.x, (int)values.y, (int)values.z, (int)values.t, minDelay, (long)timeElapsed);
  Serial.println(buf);
}
