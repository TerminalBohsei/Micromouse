#include "Sensors.h"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>   // "VL53L0X" by Pololu (Arduino Library Manager)

namespace {
enum { LEFT = 0, FRONT = 1, RIGHT = 2 };

VL53L0X       tof[3];
const int     XSHUT[3] = {PIN_XSHUT_LEFT, PIN_XSHUT_FRONT, PIN_XSHUT_RIGHT};
const uint8_t ADDR[3]  = {0x30, 0x31, 0x32};
const float   ALPHA[3] = {TOF_ALPHA_SIDE, TOF_ALPHA_FRONT, TOF_ALPHA_SIDE};
const char*   NAME[3]  = {"left", "front", "right"};
}  // namespace

bool Sensors::begin() {
  // Hold every sensor in reset, then bring them up one at a time and give
  // each its own address.
  for (int i = 0; i < 3; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
  }
  delay(20);

  bool ok = true;
  for (int i = 0; i < 3; i++) {
    pinMode(XSHUT[i], INPUT);   // release; the breakout's pull-up wakes it
    delay(15);
    tof[i].setTimeout(50);
    if (!tof[i].init()) {
      Serial.printf("ToF %s: init FAILED (check wiring / XSHUT pin)\n", NAME[i]);
      ok = false;
      continue;
    }
    tof[i].setAddress(ADDR[i]);
    tof[i].setMeasurementTimingBudget(TOF_BUDGET_US);
    tof[i].startContinuous();   // back-to-back ranging
  }
  return ok;
}

void Sensors::update() {
  const uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    // Only read when a new result is ready, so this never blocks the loop.
    if ((tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) continue;

    const uint16_t mm = tof[i].readRangeContinuousMillimeters();
    if (tof[i].timeoutOccurred()) continue;

    // 8190/8191 means "nothing in range" -> treat as far away.
    const float v = (mm >= TOF_MAX_MM) ? (float)TOF_MAX_MM : (float)mm;
    if (!seeded_[i]) {
      filt_[i]   = v;
      seeded_[i] = true;
    } else {
      filt_[i] += ALPHA[i] * (v - filt_[i]);
    }
    lastFresh_[i] = now;
  }
}

Ranges Sensors::ranges() const {
  return { (int)filt_[LEFT], (int)filt_[FRONT], (int)filt_[RIGHT] };
}

bool Sensors::fresh(uint32_t maxAgeMs) const {
  const uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    if (!seeded_[i] || (now - lastFresh_[i]) > maxAgeMs) return false;
  }
  return true;
}
