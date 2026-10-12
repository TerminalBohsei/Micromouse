#pragma once

// Two DC motors on an L298N.
class Motors {
 public:
  void begin();
  // Speeds in -255..255. Positive = forward. Applies dead-band compensation
  // and per-side trim.
  void drive(float left, float right);
  void stop();    // coast
  void brake();   // short the motor terminals (hard stop)
};
