#ifndef DRIVEWAY_MONITOR_VIBRATION_H__
#define DRIVEWAY_MONITOR_VIBRATION_H__

namespace driveway {

class VibrationSensor {
private:
  const uint8_t pin;
  volatile static long spinner;

  const uint32_t vibrationLatchInterval_ms = 15000;
  bool vibrationLatched = false;
  elapsedMillis vibrationLatchedSince;

  static void vibrationSensorInterruptHandler() {
    spinner++;
  }

public:

  byte getRawCount() const { long v = 0; noInterrupts(); v = spinner; interrupts(); return v; }
  void setup();
  long poll();
  bool latched() const { return vibrationLatched; }
  
  // This is a bit dodgy in that there is really a singleton but we haven't bothered enforcing that in code
  VibrationSensor(uint8_t pin)
  : pin(pin)
  { }
};

volatile long VibrationSensor::spinner = 0;

void VibrationSensor::setup() {
  pinMode(pin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(pin), &VibrationSensor::vibrationSensorInterruptHandler, FALLING);
}

// return (true) if any hits since we last checked
// caveat - if exactly max_of(long) will return incorrect result...
// we are polling at approx 12 Hz so a back of envelope calculation implies 
// shaking the thing at 24Hz for 2billion/24 seconds or ~ 3 years for it to miss one
long VibrationSensor::poll() {
  static byte lastVibration = 0;
  noInterrupts();
  byte v = VibrationSensor::spinner;
  interrupts();
  if (v != lastVibration) {
    lastVibration = v;
    if (!vibrationLatched) {
      Serial.println("Vibration 1");
    }
    vibrationLatched = true;
    vibrationLatchedSince = 0;
    return v;
  } else {
    if (vibrationLatchedSince > vibrationLatchInterval_ms) {
      Serial.println("Vibration 0");
      vibrationLatched = false;
    }
  }
  return 0;
}

}

#endif
