#ifndef DRIVEWAY_MONITOR_DETECTOR_H__
#define DRIVEWAY_MONITOR_DETECTOR_H__

extern elapsedMillis uptime;


struct Detector_t {
  int idx;
  int threshold;
  uint16_t dwellLength;
  uint16_t dwellCount;
  float dwellAggregate;
  float recentAverage;
  float stableAverage;

  uint16_t tentativeDetection;
  uint16_t antiDetection;
  uint16_t samplesSinceDetection;
  uint16_t continuousDetectingDwells; // used to detect "permanent" background change

  uint16_t id;
  long lastDetectionStart;
  uint16_t lastDetectionDuration;
  float variationIntegral;

  bool detectionInBlock;

  Detector_t()
  : idx(0),
    threshold(1),
    dwellLength(22), // ~2 second blocks
    dwellCount(0),
    dwellAggregate(0.F),
    recentAverage(-1.F),
    stableAverage(-1.F),
    tentativeDetection(0),
    antiDetection(0),
    continuousDetectingDwells(0),
    id(0),
    lastDetectionStart(0),
    lastDetectionDuration(0),
    variationIntegral(0.F),
    detectionInBlock(false)
  { }
};

Detector_t DetectorStatus;

bool stepDetector() {
  // Algorithm
  // - non-overlapt "integration" by averaging a block of samples
  // - if we get a spike above or below the previous average then probably a detection
  // - alternative - augment using differential instead

  // DEBUG("%d %d\n\r", DetectorStatus.idx, (int)DetectorStatus.dwellAggregate);
  bool transmitted = false;
  float m = MlxStatus.magnitude;
  DetectorStatus.idx ++;
  DetectorStatus.dwellAggregate += m;

  if (DetectorStatus.idx % DetectorStatus.dwellLength == 0) {
    DetectorStatus.dwellCount ++;
    if (DetectorStatus.dwellCount < 5) {
      // Compute a longer average when booted, to stabilise - aggregate over first 5 dwells of ~22 samples (~10 seconds)
      return false;
    } else {
      float average = DetectorStatus.dwellAggregate / DetectorStatus.idx;
      DetectorStatus.recentAverage = average;
      DetectorStatus.dwellAggregate = 0;
      DetectorStatus.idx = 0;
      if (DetectorStatus.stableAverage < 0) {
        DetectorStatus.stableAverage = average;
        Serial.print(F("stable-average,"));
        Serial.print(DetectorStatus.stableAverage);
        Serial.println();
        return false;
      }
      // Update average if there has been no detection in this block
      if (!DetectorStatus.detectionInBlock) {
        DetectorStatus.stableAverage = average;
        Serial.print(F("stable-average,"));
        Serial.print(DetectorStatus.stableAverage);
        Serial.println();
      } else {
        DetectorStatus.continuousDetectingDwells ++;
        if (DetectorStatus.continuousDetectingDwells > 20) {
          // est. 3 minutes...
          Serial.print(F("Extended detection period... update stable background"));
          DetectorStatus.stableAverage = average;
          Serial.print(F("new-stable-average,"));
          Serial.print(DetectorStatus.stableAverage);
          Serial.println();
          DetectorStatus.continuousDetectingDwells = 0;
        }
      }
    }
  }
  if (DetectorStatus.stableAverage < 0) { return false; }
  // compare sample against the stable average
  float variance = fabs(m - DetectorStatus.stableAverage);
  if (variance >= DetectorStatus.threshold) {
    // tentative detection
    DetectorStatus.tentativeDetection ++;
    if (DetectorStatus.tentativeDetection == 2) {
      // detection...
      // go until a double 0
      Serial.print(F("Detection-confirmed,"));
      Serial.print(DetectorStatus.stableAverage);
      Serial.print(',');
      Serial.print(m);
      Serial.println();
      DetectorStatus.id ++;
      DetectorStatus.lastDetectionStart = uptime;
      DetectorStatus.continuousDetectingDwells ++;
      DetectorStatus.variationIntegral = 0;
    }
    if (DetectorStatus.tentativeDetection > 1) {
      DetectorStatus.antiDetection = 0;
      DetectorStatus.detectionInBlock = true;
    }
  } else {
    DetectorStatus.antiDetection ++;
    
    // Debouncing
    // We allow ....101 as a detection but not ...100
    if (DetectorStatus.tentativeDetection == 1 && DetectorStatus.antiDetection > 1) {
      Serial.println(F("False-alarm"));
      DetectorStatus.tentativeDetection = 0;
      DetectorStatus.antiDetection = 0;
    }
    else if (DetectorStatus.tentativeDetection > 1 && DetectorStatus.antiDetection > 1) {
      Serial.println(F("Detection-completed"));
      DetectorStatus.lastDetectionDuration = uptime - DetectorStatus.lastDetectionStart;
      DetectorStatus.tentativeDetection = 0;
      DetectorStatus.antiDetection = 0;
      DetectorStatus.samplesSinceDetection = 0;
    }
  }
  if (DetectorStatus.detectionInBlock && DetectorStatus.tentativeDetection > 0) {
    DetectorStatus.variationIntegral += m;
    Serial.print(F("Det:")); Serial.print(m);
    Serial.print(F(", Int:")); Serial.println(DetectorStatus.variationIntegral);
    // schedule this outside... transmitDetection();
    transmitted = true; // indicate to loop to not call delay()
  }
  if (DetectorStatus.detectionInBlock) {
    DetectorStatus.samplesSinceDetection ++;
  }
  if (DetectorStatus.samplesSinceDetection > DetectorStatus.dwellLength * 5) {
    Serial.println(F("Detection-cleared"));
    DetectorStatus.detectionInBlock = false; // allow stable average to update again after 5 more dwells
    DetectorStatus.samplesSinceDetection = 0;
    DetectorStatus.tentativeDetection = 0;
    transmitted = false;
  }
  return transmitted;
}

#endif
