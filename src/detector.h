#ifndef DRIVEWAY_MONITOR_DETECTOR_H__
#define DRIVEWAY_MONITOR_DETECTOR_H__

extern elapsedMillis uptime;

class DetectorModel {
private:
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

public:
  long getLastDetectionStart() const { return lastDetectionStart; } 
  uint16_t getLastDetectionDuration() const { return lastDetectionDuration; }
  float getDetectionIntegral() const { return variationIntegral; }
  uint16_t getStableAverage() const { return stableAverage; }
  uint16_t getLastId() const { return id; }

  bool next();
  void setThreshold(int threshold) { this->threshold = threshold; }

  DetectorModel()
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

bool DetectorModel::next() {
  // Algorithm
  // - non-overlapt "integration" by averaging a block of samples
  // - if we get a spike above or below the previous average then probably a detection
  // - alternative - augment using differential instead

  // DEBUG("%d %d\n\r", this->idx, (int)this->dwellAggregate);
  bool transmitted = false;
  float m = MlxStatus.magnitude;
  this->idx ++;
  this->dwellAggregate += m;

  if (this->idx % this->dwellLength == 0) {
    this->dwellCount ++;
    if (this->dwellCount < 5) {
      // Compute a longer average when booted, to stabilise - aggregate over first 5 dwells of ~22 samples (~10 seconds)
      return false;
    } else {
      float average = this->dwellAggregate / this->idx;
      this->recentAverage = average;
      this->dwellAggregate = 0;
      this->idx = 0;
      if (this->stableAverage < 0) {
        this->stableAverage = average;
        Serial.print(F("stable-average,"));
        Serial.print(this->stableAverage);
        Serial.println();
        return false;
      }
      // Update average if there has been no detection in this block
      if (!this->detectionInBlock) {
        this->stableAverage = average;
        Serial.print(F("stable-average,"));
        Serial.print(this->stableAverage);
        Serial.println();
      } else {
        this->continuousDetectingDwells ++;
        if (this->continuousDetectingDwells > 20) {
          // est. 3 minutes...
          Serial.print(F("Extended detection period... update stable background"));
          this->stableAverage = average;
          Serial.print(F("new-stable-average,"));
          Serial.print(this->stableAverage);
          Serial.println();
          this->continuousDetectingDwells = 0;
        }
      }
    }
  }
  if (this->stableAverage < 0) { return false; }
  // compare sample against the stable average
  float variance = fabs(m - this->stableAverage);
  if (variance >= this->threshold) {
    // tentative detection
    this->tentativeDetection ++;
    if (this->tentativeDetection == 2) {
      // detection...
      // go until a double 0
      Serial.print(F("Detection-confirmed,"));
      Serial.print(this->stableAverage);
      Serial.print(',');
      Serial.print(m);
      Serial.println();
      this->id ++;
      this->lastDetectionStart = uptime;
      this->continuousDetectingDwells ++;
      this->variationIntegral = 0;
    }
    if (this->tentativeDetection > 1) {
      this->antiDetection = 0;
      this->detectionInBlock = true;
    }
  } else {
    this->antiDetection ++;
    
    // Debouncing
    // We allow ....101 as a detection but not ...100
    if (this->tentativeDetection == 1 && this->antiDetection > 1) {
      Serial.println(F("False-alarm"));
      this->tentativeDetection = 0;
      this->antiDetection = 0;
    }
    else if (this->tentativeDetection > 1 && this->antiDetection > 1) {
      Serial.println(F("Detection-completed"));
      this->lastDetectionDuration = uptime - this->lastDetectionStart;
      this->tentativeDetection = 0;
      this->antiDetection = 0;
      this->samplesSinceDetection = 0;
    }
  }
  if (this->detectionInBlock && this->tentativeDetection > 0) {
    this->variationIntegral += m;
    Serial.print(F("Det:")); Serial.print(m);
    Serial.print(F(", Int:")); Serial.println(this->variationIntegral);
    // schedule this outside... transmitDetection();
    transmitted = true; // indicate to loop to not call delay()
  }
  if (this->detectionInBlock) {
    this->samplesSinceDetection ++;
  }
  if (this->samplesSinceDetection > this->dwellLength * 5) {
    Serial.println(F("Detection-cleared"));
    this->detectionInBlock = false; // allow stable average to update again after 5 more dwells
    this->samplesSinceDetection = 0;
    this->tentativeDetection = 0;
    transmitted = false;
  }
  return transmitted;
}

#endif
