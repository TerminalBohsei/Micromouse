#pragma once
#include <stdint.h>
#include "Decision.h"
#include "RangeFilter.h"

// Three Time-of-Flight sensors (left, front, right). Each may be a VL53L0X or
// a VL53L1X: the type is detected at boot.
class Sensors {
 public:
  bool begin();                         // Wire must already be started
  void update();                        // non-blocking, call every loop
  Ranges ranges() const;                // filtered, clamped to TOF_MAX_MM
  bool fresh(uint32_t maxAgeMs) const;  // every sensor produced data recently?
  void restart();                       // drop old samples (call after a turn)
  const char* model(int i) const;       // 0 = left, 1 = front, 2 = right
  // Called at the first failed read and again before a bus reset: the robot uses it to brake.
  void (*beforeRecover)() = nullptr;
  uint32_t recoveries() const { return recoveries_; }   // how often the I2C bus had to be reset

 private:
  RangeFilter filt_[3]     = {RangeFilter(0.60f), RangeFilter(0.70f), RangeFilter(0.60f)};
  uint32_t    lastFresh_[3] = {0, 0, 0};
  uint32_t    lastPoll_[3]  = {0, 0, 0};
  uint8_t     kind_[3]      = {0, 0, 0};

  // I2C fault handling: a failed read is NOT a measurement. After a few in a
  // row (or silence), the bus is cleared and every sensor is re-initialised.
  uint8_t     fails_[3]     = {0, 0, 0};
  uint32_t    lastRecover_  = 0;
  uint32_t    recoveries_   = 0;
  bool        needRecover_  = false;
  bool        scanned_      = false;   // bus check already printed for this dropout
  void noteFail(int i);
  void recover();
};