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
    DETECTION = 0x01,
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
  LoraMessage detection();
#if DRIVEWAY_DATA_COLLECTOR_ENABLED
  LoraMessage debugCollection();
#endif
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
    .addUint16(CLAMP_MAX(detector.getStableAverage(), 65535))
    ;
  counter ++;
  return message;
}

template<typename Board, typename MlxSensor, typename Detector>
LoraMessage Protocol<Board, MlxSensor, Detector>::detection() {
  static long counter = 0;
  LoraMessage message;
  message
    .addUint8(MAGIC)
    .addUint8(DETECTION)
    .addUint16(counter & 0xffff)
    .addUnixtime(uptime)
    .addUint16(detector.getLastId())
    .addUnixtime(detector.getLastDetectionStart())
    .addUint16(detector.getLastDetectionDuration() / 100)
    .addUint16(CLAMP_MAX(detector.getDetectionIntegral() / 100, 65535))
    .addUint16(CLAMP_MAX(detector.getStableAverage(), 65535))
    ;
  counter ++;
  return message;
}

#if DRIVEWAY_DATA_COLLECTOR_ENABLED
#if 0
void transmitDebugCollectionFrame() {
  static long counter = 0;
  byte packet[16];
  const long tEvent = MlxSensor.getMeasurementTime() / 10;
  const uint16_t m = uint16_t(MlxSensor.getMagnitude());
  const int t = MlxSensor.getTemperature();
  byte n = 0;
  packet[n++] = 12;
  packet[n++] = 0x5f;  // not really sF, but hey
  packet[n++] = 1;     // this type of message
  packet[n++] = (counter >> 8) & 0xff; // auto wrap counter @ 65535 packets, just useful for detecting skip
  packet[n++] = (counter & 0xff);
  packet[n++] = tEvent & 0xff;
  packet[n++] = (tEvent >> 8) & 0xff;
  packet[n++] = (tEvent >> 16) & 0xff;   // ms into 100ths of a second --> 2^24 * 10 is ~40 hours of operation
  packet[n++] = m & 0xff;
  packet[n++] = (m >> 8) & 0xff;
  // Pack temp into 6 bits. We cant handle negative for now, but ths is for development anyway
  packet[n++] = (t & 0x3f) | ((MlxSensor.getResetCount() & 0x3) << 6);
  packet[n++] = 0;
  packet[0] = n;
  packet[n++] = 0;
  // DEBUG("transmitDebugCollectionFrame %d\n\r", counter);
  Radio.transmitPacket(packet, n);
  counter ++;
}
#endif
#endif

}

#endif
