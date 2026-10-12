#include "Gyro.h"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace {
constexpr uint8_t MPU_ADDR     = 0x68;   // AD0 pin low
constexpr uint8_t REG_PWR_MGMT = 0x6B;
constexpr uint8_t REG_CONFIG   = 0x1A;
constexpr uint8_t REG_GYRO_CFG = 0x1B;
constexpr uint8_t REG_GYRO_ZH  = 0x47;
constexpr float   LSB_PER_DPS  = 65.5f;  // +/-500 dps range
}  // namespace

bool Gyro::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Gyro::readRaw(int16_t& raw) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_ZH);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) return false;
  const uint8_t hi = Wire.read();
  const uint8_t lo = Wire.read();
  raw = (int16_t)((hi << 8) | lo);
  return true;
}

bool Gyro::begin() {
  if (!writeReg(REG_PWR_MGMT, 0x01)) {   // wake up, clock from gyro PLL
    Serial.println("MPU-6050: not responding (check wiring / address)");
    return false;
  }
  delay(100);
  writeReg(REG_GYRO_CFG, 0x08);          // +/-500 dps
  writeReg(REG_CONFIG,   0x03);          // digital low-pass ~44 Hz
  delay(50);

  // Measure the zero-rate offset. Retry if the robot was being moved.
  constexpr int N = 500;
  for (int attempt = 0; attempt < 3; attempt++) {
    double sum = 0, sumSq = 0;
    int n = 0;
    for (int i = 0; i < N; i++) {
      int16_t raw;
      if (readRaw(raw)) {
        sum   += raw;
        sumSq += (double)raw * raw;
        n++;
      }
      delay(2);
    }
    if (n < N / 2) continue;
    const double mean = sum / n;
    const double var  = sumSq / n - mean * mean;
    if (var < 900.0) {                   // std-dev < ~0.45 dps: robot is still
      bias_     = (float)mean;
      heading_  = 0;
      rate_     = 0;
      lastUs_   = micros();
      return true;
    }
    Serial.println("Gyro calibration: robot moved, retrying...");
  }
  Serial.println("Gyro calibration failed - keep the robot still at power-on");
  return false;
}

void Gyro::update() {
  const uint32_t nowUs = micros();
  const float dt = (nowUs - lastUs_) * 1e-6f;
  lastUs_ = nowUs;

  int16_t raw;
  if (!readRaw(raw) || dt <= 0.0f || dt > 0.1f) return;

  float r = GYRO_SIGN * ((float)raw - bias_) / LSB_PER_DPS;
  if (fabsf(r) < GYRO_DEADBAND_DPS) r = 0;
  rate_     = r;
  heading_ += r * dt;
}
