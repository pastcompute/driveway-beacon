#include <Arduino.h>

#define PIN_VIBRATION   5

volatile byte vibration = 0;

void vibrationSensorInterruptHandler() {
  vibration++;
}

void setup() {
  pinMode(PIN_VIBRATION, INPUT_PULLDOWN);
  delay(5000);
  Serial.println(F("go"));
  attachInterrupt(PIN_VIBRATION, vibrationSensorInterruptHandler, FALLING);
}

int n = 0;
byte b0=0, v0=0;

void loop() {
  auto b = digitalRead(PIN_VIBRATION);
  if (n == 0 || b0 != b || v0 != vibration) {
    Serial.print(n);
    Serial.print(',');
    Serial.print(b);
    Serial.print(',');
    Serial.print(vibration);
    Serial.println();
    v0 = vibration;
    b0 = b;
  }
  n++;
}