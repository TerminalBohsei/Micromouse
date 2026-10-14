#pragma once
#include <stdint.h>

// Downward-facing IR reflectance sensor (KY-033 / IR-08H style module) that
// spots the end-zone. The floor level is captured when the run starts, and the
// finish is declared when the reading stays different for IR_CONFIRM_MS.
class FloorSensor {
 public:
  void begin();                    // configure the pin
  void captureBaseline();          // robot is standing on the normal floor
  void update(uint32_t nowMs);     // call every control tick
  bool finishDetected() const { return detected_; }
  int  raw() const;                // digital: 0/1, analog: 0-4095
  int  baseline() const { return base_; }
  void clear() { detected_ = false; since_ = 0; active_ = false; }

 private:
  bool differs(int v) const;
  int      base_     = 0;
  uint32_t since_    = 0;
  bool     active_   = false;
  bool     detected_ = false;
};
