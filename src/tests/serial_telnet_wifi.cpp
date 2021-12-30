#include "ESPTelnet.h"          

class elapsedMillis
{
private:
	unsigned long ms;
public:
	elapsedMillis(void) { ms = millis(); }
	elapsedMillis(unsigned long val) { ms = millis() - val; }
	elapsedMillis(const elapsedMillis &orig) { ms = orig.ms; }
	operator unsigned long () const { return millis() - ms; }
	elapsedMillis & operator = (const elapsedMillis &rhs) { ms = rhs.ms; return *this; }
	elapsedMillis & operator = (unsigned long val) { ms = millis() - val; return *this; }
	elapsedMillis & operator -= (unsigned long val)      { ms += val ; return *this; }
	elapsedMillis & operator += (unsigned long val)      { ms -= val ; return *this; }
	elapsedMillis operator - (int val) const           { elapsedMillis r(*this); r.ms += val; return r; }
	elapsedMillis operator - (unsigned int val) const  { elapsedMillis r(*this); r.ms += val; return r; }
	elapsedMillis operator - (long val) const          { elapsedMillis r(*this); r.ms += val; return r; }
	elapsedMillis operator - (unsigned long val) const { elapsedMillis r(*this); r.ms += val; return r; }
	elapsedMillis operator + (int val) const           { elapsedMillis r(*this); r.ms -= val; return r; }
	elapsedMillis operator + (unsigned int val) const  { elapsedMillis r(*this); r.ms -= val; return r; }
	elapsedMillis operator + (long val) const          { elapsedMillis r(*this); r.ms -= val; return r; }
	elapsedMillis operator + (unsigned long val) const { elapsedMillis r(*this); r.ms -= val; return r; }
	unsigned long get() const { return ms; }
};


/* ------------------------------------------------- */

#define SERIAL_SPEED    115200
#define WIFI_SSID       "dropbear"
#define WIFI_PASSWORD   "yesterda7"

#define DEBUGS 0

/* ------------------------------------------------- */

ESPTelnet telnet;
IPAddress ip;

/* ------------------------------------------------- */

elapsedMillis lastRx;
elapsedMillis lastTx;

void setupSerial(long speed, String msg = "") {
  Serial.begin(speed);
  while (!Serial) {
  }
  delay(200);
#if DEBUGS
  Serial.println();
  Serial.println();
  if (msg != "") Serial.println(msg);
#endif
}

/* ------------------------------------------------- */

bool isConnected() {
  return (WiFi.status() == WL_CONNECTED);
}

/* ------------------------------------------------- */

bool connectToWiFi(const char* ssid, const char* password, int max_tries = 20, int pause = 500) {
  int i = 0;
  WiFi.mode(WIFI_STA);
  #if defined(ARDUINO_ARCH_ESP8266)
    WiFi.forceSleepWake();
    delay(200);
  #endif
  WiFi.begin(ssid, password);
  do {
    delay(pause);
#if DEBUGS
    Serial.print(".");
#endif
  } while (!isConnected() || i++ < max_tries);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  return isConnected();
}

/* ------------------------------------------------- */

void errorMsg(String error, bool restart = true) {
#if DEBUGS
  Serial.println(error);
#endif
  if (restart) {
#if DEBUGS
    Serial.println("Rebooting now...");
#endif
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

/* ------------------------------------------------- */

extern void onTelnetConnect(String ip);
extern void onTelnetConnectionAttempt(String ip);
extern void onTelnetReconnect(String ip);
extern void onTelnetDisconnect(String ip);


void setupTelnet() {  
  // passing on functions for various telnet events
  telnet.onConnect(&onTelnetConnect);
  telnet.onConnectionAttempt(&onTelnetConnectionAttempt);
  telnet.onReconnect(&onTelnetReconnect);
  telnet.onDisconnect(&onTelnetDisconnect);
  
  // passing a lambda function
  telnet.onInputReceived([](String str) {
    // checks for a certain command
#if LEDS_WORK
    digitalWrite(LED_BUILTIN, LOW);
#endif
    if (str == "ping") {
      telnet.println("> pong");
#if DEBUGS
      Serial.println("- Telnet: pong");
#endif
    }
    Serial.print(str);
    lastTx = 0;
  });

#if DEBUGS
  Serial.print("- Telnet: ");
#endif
  if (telnet.begin()) {
#if DEBUGS
    Serial.println("running");
#endif
  } else {
#if DEBUGS
    Serial.println("error.");
#endif
    errorMsg("Will reboot...");
  }
}

/* ------------------------------------------------- */


// (optional) callback functions for telnet events
void onTelnetConnect(String ip) {
#if LEDS_WORK
  digitalWrite(LED_BUILTIN, HIGH);
#endif
#if DEBUGS
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
#endif
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("(Use ^] + q  to disconnect.)");
}

void onTelnetDisconnect(String ip) {
#if DEBUGS
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
#endif
}

void onTelnetReconnect(String ip) {
#if DEBUGS
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
#endif
}

void onTelnetConnectionAttempt(String ip) {
#if DEBUGS
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connected");
#endif
}

/* ------------------------------------------------- */

void setup() {
#if LEDS_WORK
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif
  setupSerial(SERIAL_SPEED, "Telnet Test");
  
#if DEBUGS
  Serial.print("- Wifi: ");
#endif
  connectToWiFi(WIFI_SSID, WIFI_PASSWORD);
  
  if (isConnected()) {
    ip = WiFi.localIP();
#if DEBUGS
    Serial.print(" ");
    Serial.println(ip);
#endif
    setupTelnet();
  } else {
#if DEBUGS
    Serial.println();
    errorMsg("Error connecting to WiFi");
#endif
  }
}

/* ------------------------------------------------- */

void loop() {
  telnet.loop();

  // send serial input to telnet as output
  if (Serial.available()) {
#if LEDS_WORK
    digitalWrite(LED_BUILTIN, LOW);
#endif
    telnet.print(Serial.read());
    lastRx = 0;
  }
  // and telnet data back to serial

  if (lastRx > 1000 && lastTx > 1000) {
#if LEDS_WORK
    digitalWrite(LED_BUILTIN, HIGH);
#endif
  }
}
//* ------------------------------------------------- */
