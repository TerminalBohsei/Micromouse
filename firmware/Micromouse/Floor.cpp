#include "Floor.h"
#include "Config.h"
#include <Arduino.h>
#include <stdlib.h>

void FloorSensor::begin() { pinMode(PIN_IR, INPUT); }

int FloorSensor::raw() const {
  return IR_ANALOG ? analogRead(PIN_IR) : (digitalRead(PIN_IR) ? 1 : 0);
}

void FloorSensor::captureBaseline() {
  clear();
  if (IR_ANALOG) {
    long sum = 0;
    for (int i = 0; i < 32; i++) { sum += raw(); delay(2); }
    base_ = (int)(sum / 32);
  } else {                           // majority vote of 31 samples
    int ones = 0;
    for (int i = 0; i < 31; i++) { ones += raw(); delay(2); }
    base_ = ones > 15 ? 1 : 0;
  }
}

bool FloorSensor::differs(int v) const {
  if (!IR_ANALOG && IR_FINISH_LEVEL >= 0) return v == IR_FINISH_LEVEL;
  return IR_ANALOG ? (abs(v - base_) > IR_DELTA) : (v != base_);
}

void FloorSensor::update(uint32_t nowMs) {
  if (detected_) return;
  if (differs(raw())) {
    if (!active_) { active_ = true; since_ = nowMs; diffCount_ = 0; }
    diffCount_++;
    // Time AND a minimum number of consecutive samples: a flicker cannot trigger it.
    if (nowMs - since_ >= IR_CONFIRM_MS && diffCount_ >= 8) detected_ = true;
  } else {
    active_ = false;
  }
}