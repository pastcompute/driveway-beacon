/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

#if defined(XMC_BOARD)
#include <DeviceControlXMC.h>
XMCClass devCtrl;
#endif

#ifdef TEENSYDUINO
#include <InternalTemperature.h>
#endif

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
#if defined(XMC_BOARD)
  pinMode(LED2, OUTPUT);
#endif

  delay(1000);
  Serial.begin(115200);
  Serial.println(F("Hello, World"));
}

int n = 0;

void loop()
{
  int tc = 0;
#if defined(XMC_BOARD)
  tc = devCtrl.getTemperature();
#endif
#ifdef TEENSYDUINO
  tc = InternalTemperature.readTemperatureC();
#endif
  Serial.print(F("blink - LED pin ="));
  Serial.print(LED_BUILTIN);
  Serial.print(F(" - "));
  Serial.print(n);
  Serial.print(F(" - "));
  Serial.print(tc);
  Serial.print(" oC                \r");
  n++;

  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
#if defined(XMC_BOARD)
  digitalWrite(LED2, LOW);
#endif  
  delay(100);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
#if defined(XMC_BOARD)
  digitalWrite(LED2, HIGH);
#endif  
   // wait for a second
  delay(100);
#if defined(XMC_BOARD)
  digitalWrite(LED2, LOW);
#endif

  // see if reducing LED reduces temp
  delay(1000);
}
