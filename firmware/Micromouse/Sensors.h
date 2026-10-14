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

 private:
  RangeFilter filt_[3]     = {RangeFilter(0.60f), RangeFilter(0.70f), RangeFilter(0.60f)};
  uint32_t    lastFresh_[3] = {0, 0, 0};
  uint32_t    lastPoll_[3]  = {0, 0, 0};
  uint8_t     kind_[3]      = {0, 0, 0};
};
