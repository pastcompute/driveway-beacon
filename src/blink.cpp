/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);
  Serial.begin(115200);
  Serial.println(F("Hello, World"));
}

int n = 0;

void loop()
{
  Serial.print(F("blink - LED pin ="));
  Serial.print(LED_BUILTIN);
  Serial.print(F(" - "));
  Serial.print(n);
  Serial.print("                \r");
  n++;

  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
#if defined(XMC_BOARD)
  digitalWrite(LED2, LOW);
#endif  
  // wait for a second
  delay(700);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
#if defined(XMC_BOARD)
  digitalWrite(LED2, HIGH);
#endif  
   // wait for a second
  delay(300);
}
