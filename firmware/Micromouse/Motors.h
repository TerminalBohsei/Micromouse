#pragma once
#include <stdint.h>
#include "Config.h"

// Two DC motors on an L298N.
class Motors {
 public:
  void begin();
  // Speeds in -255..255. Positive = forward. Applies dead-band compensation
  // and per-side trim.
  void drive(float left, float right);
  void stop();    // coast
  void brake();   // short the motor terminals (hard stop)
  // Diagnostic: plain ON/OFF on the IN pins, no PWM at all (see Navigator 'D').
  void rawTest(uint32_t holdMs);

  // Per-side speed scale (1.0 = full). Starts at MOTOR_TRIM_LEFT / _RIGHT from
  // Config.h and can be changed live from the serial monitor (keys 1-4).
  float trimLeft()  const { return trimL_; }
  float trimRight() const { return trimR_; }
  void  setTrim(float l, float r) { trimL_ = l; trimR_ = r; }
  void  setTurnTrim(float l, float r) { turnTrimL_ = l; turnTrimR_ = r; }

 private:
  float trimL_ = MOTOR_TRIM_LEFT;
  float trimR_ = MOTOR_TRIM_RIGHT;
  float turnTrimL_ = MOTOR_TURN_TRIM_LEFT;    // used only while spinning on the spot
  float turnTrimR_ = MOTOR_TURN_TRIM_RIGHT;
};