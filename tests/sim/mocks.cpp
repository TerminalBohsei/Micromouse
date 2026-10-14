// Simulator stand-ins for the hardware-bound classes. They keep the same
// public interface as the real Sensors / Gyro / Motors, so the REAL
// Navigator.cpp runs unchanged. The ToF filtering uses the real RangeFilter.h
// and the dead-band mapping uses the real MotorMap.h.
#include <Arduino.h>
#include "Sensors.h"
#include "Gyro.h"
#include "Motors.h"
#include "MotorMap.h"
#include "Config.h"
#include "world.h"

// --------------------------------------------------------------- Arduino ----
SerialShim Serial;
unsigned long millis() { return (unsigned long)(g_world->nowUs / 1000); }
unsigned long micros() { return (unsigned long)(g_world->nowUs); }
void delay(unsigned long ms) { for (unsigned long i = 0; i < ms; i++) g_world->step(1000); }
void pinMode(int, int) {}
void digitalWrite(int, int) {}
int  analogRead(int) { return 0; }
int  digitalRead(int pin) {
  if (pin == PIN_START) return g_world->startPressed ? LOW : HIGH;
  if (pin == PIN_IR)    return g_world->irOnFinish() ? 1 : 0;      // black floor = 0
  return 0;
}

// --------------------------------------------------------------- Sensors ----
bool Sensors::begin() { return true; }
void Sensors::update() {
  const uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    if (!g_world->ready[i]) continue;
    g_world->ready[i] = false;
    const int mm = g_world->mm[i];
    filt_[i].push(mm >= TOF_MAX_MM ? TOF_MAX_MM : mm);
    lastFresh_[i] = now;
  }
}
Ranges Sensors::ranges() const { return { filt_[0].value(), filt_[1].value(), filt_[2].value() }; }
bool Sensors::fresh(uint32_t maxAgeMs) const {
  const uint32_t now = millis();
  for (int i = 0; i < 3; i++) if (lastFresh_[i] == 0 || now - lastFresh_[i] > maxAgeMs) return false;
  return true;
}
void Sensors::restart() { for (int i = 0; i < 3; i++) filt_[i].restart(); }
const char* Sensors::model(int) const { return "sim"; }

// ------------------------------------------------------------------ Gyro ----
// bias_ is kept in deg/s here.
bool Gyro::begin() { name_ = "sim"; return calibrate(0); }
bool Gyro::calibrate(uint16_t) {
  bias_ = (float)(g_world->gyroBias + 0.01 * g_world->gauss());   // what a still robot measures
  heading_ = 0; rate_ = 0; lastUs_ = micros();
  return true;
}
void Gyro::update() {
  const uint32_t nowUs = micros();
  if (nowUs - lastUs_ < 4000) return;
  float dt = (nowUs - lastUs_) * 1e-6f; lastUs_ = nowUs;
  const double raw = g_world->omegaDps + g_world->gyroBias + 0.08 * g_world->gauss();
  float r = GYRO_SIGN * (float)(raw - bias_);
  if (fabsf(r) < GYRO_DEADBAND_DPS) r = 0;
  rate_ = r; heading_ += r * dt;
}

// ---------------------------------------------------------------- Motors ----
void Motors::begin() {}
void Motors::drive(float left, float right) {
  auto one = [](float v, bool inv, float trim) {
    if (inv) v = -v;
    v = clampf(v * trim, -255.0f, 255.0f);
    return (v < 0 ? -1.0 : 1.0) * motorDuty(v, MOTOR_MIN_PWM);
  };
  g_world->dutyL = one(left,  MOTOR_LEFT_INVERT,  MOTOR_TRIM_LEFT);
  g_world->dutyR = one(right, MOTOR_RIGHT_INVERT, MOTOR_TRIM_RIGHT);
  g_world->braking = false;
}
void Motors::stop()  { drive(0, 0); }
void Motors::brake() { g_world->dutyL = g_world->dutyR = 0; g_world->braking = true; }
