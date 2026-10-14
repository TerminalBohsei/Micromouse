#pragma once
#include <stdint.h>

// 28BYJ-48 stepper on a ULN2003 board, half-stepping (4096 half-steps per
// output revolution). Non-blocking: call update() every loop. Does nothing at
// all (and never touches its pins) unless STEPPER_ENABLED is true in Config.h.
class Stepper {
 public:
  void begin();
  void moveSteps(long steps);   // + = one direction, - = the other; queued, returns at once
  void update();                // call every loop
  bool busy() const { return remaining_ != 0; }
  void release();               // all coils off (stops the motor heating up)

 private:
  void apply(uint8_t phase);
  long     remaining_ = 0;
  uint8_t  phase_     = 0;
  uint32_t lastUs_    = 0;
};
