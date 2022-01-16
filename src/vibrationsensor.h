#ifndef DRIVEWAY_MONITOR_VIBRATION_H__
#define DRIVEWAY_MONITOR_VIBRATION_H__

namespace driveway {

class VibrationSensor {
private:
  const uint8_t pin;
  volatile static long spinner;

  static void vibrationSensorInterruptHandler() {
    spinner++;
  }

public:

  byte getRawCount() const { long v = 0; noInterrupts(); v = spinner; interrupts(); return v; }
  void setup();
  long poll();

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
long VibrationSensor::poll() {
  static byte lastVibration = 0;
  noInterrupts();
  byte v = VibrationSensor::spinner;
  interrupts();
  if (v != lastVibration) {
    lastVibration = v;
    return v;
  }
  return 0;
}

}

#endif
