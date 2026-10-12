#pragma once
#include <stdint.h>
#include "Decision.h"

// Three VL53L0X Time-of-Flight sensors (left, front, right).
class Sensors {
 public:
  bool begin();                       // Wire must already be started
  void update();                      // non-blocking, call every loop
  Ranges ranges() const;              // filtered, clamped to TOF_MAX_MM
  bool fresh(uint32_t maxAgeMs) const;  // every sensor produced data recently?

 private:
  float    filt_[3]      = {0, 0, 0};
  bool     seeded_[3]    = {false, false, false};
  uint32_t lastFresh_[3] = {0, 0, 0};
};
