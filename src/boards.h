#ifndef DRIVEWAY_MONITOR_BOARDS_H__
#define DRIVEWAY_MONITOR_BOARDS_H__

// Sanity check our pio settings, because unfortunately this affects arduino main cpp
#ifdef SERIAL_DEBUG
#ifdef SERIAL_ONBOARD
#error oops1
#endif
#ifdef SERIAL_HOSTPC
#error oops2
#endif
#endif

#if defined(TEENSYDUINO)

#define PIN_VIBRATION   5 // at the moment this is the pin used for DHT which we dont need
#define LED_MAIN LED_BUILTIN
#define PIN_SX1276_RST  21
#define PIN_SX1276_MISO PIN_SPI_MISO // 12
#define PIN_SX1276_MOSI PIN_SPI_MOSI // 11
#define PIN_SX1276_CS   22 // alt pin (PIN_SPI_SS == 10)
#define PIN_SX1276_SCK  14 // alt pin (PIN_SPI_SCK == 13, == LED_BUILTIN)

#define PIN_SDA PIN_WIRE_SDA // 18
#define PIN_SCL PIN_WIRE_SCL // 19

#define MLX_IRQ 9

#define ICACHE_FLASH_ATTR

#define BOARD_NAME "TEENSYLC"

#include <InternalTemperature.h>

#else
#error "Unsupported configuration"
#endif

namespace driveway {
class Board {
public:
  void setup() {
#if defined(TEENSYDUINO)
    // prepare to read Vcc or other analog ops
    analogReference(DEFAULT);
    analogReadResolution(12);
    analogReadAveraging(32);
#endif    
  }

  float readVcc() {
    // TODO
    return 0;
  }

  float readTemperatureC() {
#if defined(TEENSYDUINO)
    return InternalTemperature.readTemperatureC();
#else
    return 0;
#endif
  }
};
}

#endif
