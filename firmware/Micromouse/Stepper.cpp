#include "Stepper.h"
#include "Config.h"
#include <Arduino.h>

namespace {
// Coil patterns for IN1..IN4 (bit0 = IN1) - half-step sequence.
const uint8_t SEQ[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};
const int     PINS[4] = {PIN_STEP_IN1, PIN_STEP_IN2, PIN_STEP_IN3, PIN_STEP_IN4};
}  // namespace

void Stepper::begin() {
  if (!STEPPER_ENABLED) return;
  for (int p : PINS) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
}

void Stepper::apply(uint8_t phase) {
  for (int b = 0; b < 4; b++) digitalWrite(PINS[b], (SEQ[phase] >> b) & 1);
}

void Stepper::release() {
  if (!STEPPER_ENABLED) return;
  for (int p : PINS) digitalWrite(p, LOW);
}

void Stepper::moveSteps(long steps) {
  if (!STEPPER_ENABLED) return;
  remaining_ = steps;
  lastUs_    = micros();
}

void Stepper::update() {
  if (!STEPPER_ENABLED || remaining_ == 0) return;
  const uint32_t now = micros();
  if (now - lastUs_ < STEPPER_STEP_US) return;
  lastUs_ = now;

  if (remaining_ > 0) { phase_ = (uint8_t)((phase_ + 1) & 7); remaining_--; }
  else                { phase_ = (uint8_t)((phase_ + 7) & 7); remaining_++; }
  apply(phase_);
  if (remaining_ == 0) release();
}
