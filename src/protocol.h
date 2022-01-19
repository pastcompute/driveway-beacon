#ifndef DRIVEWAY_MONITOR_PROTOCOL_H__
#define DRIVEWAY_MONITOR_PROTOCOL_H__

#include "LoraMessage.h"

extern elapsedMillis uptime;

namespace driveway {

template<typename Board, typename MlxSensor, typename Detector>
class Protocol {
private:
  enum {
    MAGIC = 0x5f,
    HEARTBEAT = 0x02,
  };

  Board& board;
  MlxSensor& mlx;
  Detector& detector;

  uint16_t CLAMP_MAX(float v, uint16_t m) { return (v < m)?v:m; }
public:
  Protocol(Board& board, MlxSensor& mlx, Detector& detector)
  : board(board),
    mlx(mlx),
    detector(detector)
  { }
  LoraMessage heartbeat();
};

template<typename Board, typename MlxSensor, typename Detector>
LoraMessage Protocol<Board, MlxSensor, Detector>::heartbeat() {
  static long counter = 0;
  LoraMessage message;
  message
    .addUint8(MAGIC)
    .addUint8(HEARTBEAT)
    .addUint16(counter & 0xffff)
    .addUnixtime(uptime)
    .addTemperature(board.readTemperatureC())
    .addUnixtime(mlx.getMeasurementTime())
    .addUint16(CLAMP_MAX(mlx.getMagnitude(), 9999))
    .addUint16(CLAMP_MAX(detector.getStableAverage(), 9999))
    ;
  counter ++;
  return message;
}

}

#endif
