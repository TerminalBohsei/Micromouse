#include "Sensors.h"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdio.h>
#include <VL53L0X.h>                       // "VL53L0X" by Pololu
#if TOF_ENABLE_VL53L1X
#include <VL53L1X.h>                       // "VL53L1X" by Pololu
#endif

namespace {
enum { LEFT = 0, FRONT = 1, RIGHT = 2 };
enum Kind : uint8_t { NONE = 0, L0X = 1, L1X = 2 };

VL53L0X       tof0[3];
#if TOF_ENABLE_VL53L1X
VL53L1X       tof1[3];
#endif
const int     XSHUT[3] = {PIN_XSHUT_LEFT, PIN_XSHUT_FRONT, PIN_XSHUT_RIGHT};
const uint8_t ADDR[3]  = {0x30, 0x31, 0x32};
const char*   NAME[3]  = {"left", "front", "right"};
constexpr uint32_t POLL_MS = 4;            // check "data ready" at most this often

// Result of one measurement.
enum class Sample : uint8_t { SKIP, FAR, VALUE };

// A failed I2C read makes the Pololu library return 0xFFFF. A real "nothing in
// range" reading from a VL53L0X is 8190/8191, so anything above this is an error.
constexpr uint16_t MAX_REAL_RANGE = 8192;
constexpr uint8_t  FAILS_BEFORE_RECOVER = 3;     // bad reads in a row
constexpr uint32_t SILENT_BEFORE_RECOVER_MS = 200;   // no good sample from a sensor for this long
constexpr uint32_t RECOVER_COOLDOWN_MS = 400;

bool expected(int i) {
#if BENCH_FRONT_ONLY
  return i == FRONT;
#else
  (void)i;
  return true;
#endif
}

// A sensor can hold SDA low after a glitch, which locks the whole bus.
// Clock SCL until it lets go, then send a STOP.
bool busClear() {
  auto low  = [](int pin) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); };
  auto free = [](int pin) { pinMode(pin, INPUT_PULLUP); };
  free(PIN_SDA); free(PIN_SCL);
  delayMicroseconds(10);
  const bool stuck = digitalRead(PIN_SDA) == LOW;
  for (int n = 0; n < 9 && digitalRead(PIN_SDA) == LOW; n++) {
    low(PIN_SCL);  delayMicroseconds(10);
    free(PIN_SCL); delayMicroseconds(10);
  }
  low(PIN_SDA);  delayMicroseconds(10);        // STOP: SDA rises while SCL is high
  free(PIN_SCL); delayMicroseconds(10);
  free(PIN_SDA); delayMicroseconds(10);
  return stuck;
}

// Which sensors still answer on the bus? Tells a power/wiring fault from a glitch.
void scanSensors() {
  const uint8_t addrs[] = {0x29, ADDR[0], ADDR[1], ADDR[2]};
  char buf[64]; buf[0] = 0;
  int n = 0;
  for (uint8_t a : addrs) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) n += snprintf(buf + n, sizeof(buf) - n, " 0x%02X", a);
  }
  Serial.printf("I2C check: answering at:%s\n", n ? buf : " nothing");
  if (!n) Serial.println("  -> nothing answers: the sensors have no 3.3V/GND, or SDA/SCL is broken (wiggle those 4 wires).");
  else if (strstr(buf, "0x29")) Serial.println("  -> sensors answer at 0x29: they lost power for a moment and reset (3.3V dip).");
}
}  // namespace

void Sensors::noteFail(int i) {
  if (fails_[i] == 1 && beforeRecover) beforeRecover();   // 2nd bad read in a row: stop the wheels
  if (fails_[i] < 255) fails_[i]++;
  if (fails_[i] >= FAILS_BEFORE_RECOVER) needRecover_ = true;
}

// Reset the I2C bus and bring all three sensors back up (XSHUT reset + new addresses).
void Sensors::recover() {
  if (beforeRecover) beforeRecover();           // never reset the bus with the wheels running
  recoveries_++;
  Serial.printf("ToF I2C problem - resetting the bus and the sensors (#%lu)\n", (unsigned long)recoveries_);
  Wire.end();
  const bool stuck = busClear();
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(TOF_I2C_HZ);
  Wire.setTimeOut(20);
  if (!scanned_) {                                  // report once per dropout
    scanned_ = true;
    Serial.printf("I2C check: SDA held low by a device: %s\n", stuck ? "YES" : "no");
    scanSensors();
  }
  const bool ok = begin();
  for (int i = 0; i < 3; i++) fails_[i] = 0;
  needRecover_ = false;
  lastRecover_ = millis();
  Serial.println(ok ? "ToF sensors back." : "ToF sensors still missing - will try again.");
}

bool Sensors::begin() {
  // Hold every sensor in reset, then bring them up one at a time and give
  // each its own address (they all power up at 0x29).
  for (int i = 0; i < 3; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
    kind_[i] = NONE;
  }
  delay(20);

  bool ok = true;
  for (int i = 0; i < 3; i++) {
#if BENCH_FRONT_ONLY
    // Left/right are not fitted: leave them in reset so only the front sensor
    // answers on the bus (otherwise it could be mistaken for "left").
    if (i != FRONT) continue;
#endif
    pinMode(XSHUT[i], INPUT);              // release; the breakout's pull-up wakes it
    delay(15);

    tof0[i].setTimeout(50);
    if (tof0[i].init()) {                  // checks the model ID: true only for a VL53L0X
      tof0[i].setAddress(ADDR[i]);
      tof0[i].setMeasurementTimingBudget(TOF_BUDGET_US);
      tof0[i].startContinuous();           // back-to-back ranging
      kind_[i] = L0X;
      continue;
    }
#if TOF_ENABLE_VL53L1X
    tof1[i].setTimeout(50);
    if (tof1[i].init()) {                  // checks the model ID: true only for a VL53L1X
      tof1[i].setAddress(ADDR[i]);
      tof1[i].setDistanceMode(VL53L1X::Short);   // best ambient-light immunity, 1.3 m
      tof1[i].setMeasurementTimingBudget(TOF_BUDGET_US);
      tof1[i].startContinuous(TOF_BUDGET_US / 1000);
      kind_[i] = L1X;
      continue;
    }
#endif
    Serial.printf("ToF %s: not found (check wiring, 3.3 V and its XSHUT pin)\n", NAME[i]);
    ok = false;
  }
  for (int i = 0; i < 3; i++)
    if (kind_[i]) Serial.printf("ToF %s: %s at 0x%02X\n", NAME[i], model(i), ADDR[i]);
  lastRecover_ = millis();
  return ok;
}

void Sensors::update() {
  const uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    if (!kind_[i] || (now - lastPoll_[i]) < POLL_MS) continue;
    lastPoll_[i] = now;

    Sample s = Sample::SKIP;
    int mm = 0;

    if (kind_[i] == L0X) {
      // Only read when a new result is ready, so this never blocks the loop.
      const uint8_t rdy = tof0[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
      if (tof0[i].last_status != 0) { noteFail(i); continue; }      // I2C write failed
      if ((rdy & 0x07) == 0) continue;
      const uint16_t r = tof0[i].readRangeContinuousMillimeters();
      if (tof0[i].timeoutOccurred() || tof0[i].last_status != 0 || r >= MAX_REAL_RANGE) {
        noteFail(i);                       // bad I2C read: NOT "far away"
        continue;
      }
      // 8190/8191 means "nothing in range" -> treat as far away.
      if (r >= TOF_MAX_MM) s = Sample::FAR;
      else { s = Sample::VALUE; mm = r; }
    }
#if TOF_ENABLE_VL53L1X
    else if (kind_[i] == L1X) {
      const bool rdy = tof1[i].dataReady();
      if (tof1[i].last_status != 0) { noteFail(i); continue; }
      if (!rdy) continue;
      const uint16_t r = tof1[i].read(false);
      if (tof1[i].timeoutOccurred() || tof1[i].last_status != 0 || r == 0xFFFF) { noteFail(i); continue; }
      switch (tof1[i].ranging_data.range_status) {
        case VL53L1X::RangeValid:
        case VL53L1X::RangeValidMinRangeClipped:
        case VL53L1X::RangeValidNoWrapCheckFail:
          if (r >= TOF_MAX_MM) s = Sample::FAR; else { s = Sample::VALUE; mm = r; }
          break;
        case VL53L1X::SigmaFail:           // too noisy / too weak: nothing useful ahead
        case VL53L1X::SignalFail:
        case VL53L1X::OutOfBoundsFail:
        case VL53L1X::WrapTargetFail:
          s = Sample::FAR; break;
        case VL53L1X::MinRangeFail:        // target closer than the minimum
          s = Sample::VALUE; mm = 15; break;
        default: break;                    // hardware fail / sync: ignore the sample
      }
    }
#endif
    if (s == Sample::SKIP) continue;
    filt_[i].push(s == Sample::FAR ? TOF_MAX_MM : mm);
    lastFresh_[i] = now;
    fails_[i] = 0;
    scanned_ = false;
  }

  // Recover if reads keep failing, a sensor went missing, or one has been silent too long.
  bool bad = needRecover_;
  for (int i = 0; i < 3 && !bad; i++) {
    if (!expected(i)) continue;
    const uint32_t ref = lastFresh_[i] ? lastFresh_[i] : lastRecover_;
    if (!kind_[i] || (now - ref) > SILENT_BEFORE_RECOVER_MS) bad = true;
  }
  if (bad && (now - lastRecover_) >= RECOVER_COOLDOWN_MS) recover();
}

Ranges Sensors::ranges() const {
  return { filt_[LEFT].value(), filt_[FRONT].value(), filt_[RIGHT].value() };
}

bool Sensors::fresh(uint32_t maxAgeMs) const {
  const uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    if (!kind_[i]) {
#if BENCH_FRONT_ONLY
      continue;                          // sensor not fitted in bench mode
#else
      return false;
#endif
    }
    // lastFresh_ == 0 means this sensor has never produced a sample.
    if (lastFresh_[i] == 0 || (now - lastFresh_[i]) > maxAgeMs) return false;
  }
  return true;
}

void Sensors::restart() {
  for (int i = 0; i < 3; i++) filt_[i].restart();
}

const char* Sensors::model(int i) const {
  return kind_[i] == L0X ? "VL53L0X" : (kind_[i] == L1X ? "VL53L1X" : "none");
}