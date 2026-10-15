#include "Gyro.h"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

namespace {
constexpr float    LSB_PER_DPS = 65.5f;   // both families at +/-500 dps full scale
constexpr uint32_t POLL_US     = 4000;    // read the gyro at most every 4 ms (250 Hz)

// ---- MPU-6050 / 6500 / 9250 register map ----
constexpr uint8_t MPU_SMPLRT_DIV   = 0x19;
constexpr uint8_t MPU_CONFIG       = 0x1A;
constexpr uint8_t MPU_GYRO_CONFIG  = 0x1B;
constexpr uint8_t MPU_GYRO_ZOUT_H  = 0x47;
constexpr uint8_t MPU_PWR_MGMT_1   = 0x6B;
constexpr uint8_t MPU_PWR_MGMT_2   = 0x6C;
constexpr uint8_t MPU_WHO_AM_I     = 0x75;

// ---- ICM-20948 register map (bank 0 unless noted) ----
constexpr uint8_t ICM_WHO_AM_I     = 0x00;   // reads 0xEA
constexpr uint8_t ICM_PWR_MGMT_1   = 0x06;
constexpr uint8_t ICM_PWR_MGMT_2   = 0x07;
constexpr uint8_t ICM_GYRO_ZOUT_H  = 0x37;
constexpr uint8_t ICM_REG_BANK_SEL = 0x7F;
constexpr uint8_t ICM_B2_GYRO_SMPLRT_DIV = 0x00;  // bank 2
constexpr uint8_t ICM_B2_GYRO_CONFIG_1   = 0x01;  // bank 2
}  // namespace

bool Gyro::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool Gyro::readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom(addr_, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) {
    if (!Wire.available()) return false;
    buf[i] = (uint8_t)Wire.read();
  }
  return true;
}

bool Gyro::readRaw(int16_t& raw) {
  uint8_t b[2];
  if (!readRegs(kind_ == Kind::ICM ? ICM_GYRO_ZOUT_H : MPU_GYRO_ZOUT_H, b, 2)) return false;
  raw = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  return raw != -32768;                                  // -32768 = clipped / invalid
}

// ------------------------------------------------------------- detection ---
bool Gyro::detect() {
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    addr_ = a;
    uint8_t id = 0;

    // InvenSense MPU family: WHO_AM_I at 0x75.
    kind_ = Kind::MPU;
    if (readRegs(MPU_WHO_AM_I, &id, 1)) {
      switch (id) {
        case 0x68: name_ = "MPU-6050"; return true;
        case 0x70: name_ = "MPU-6500"; return true;
        case 0x71: name_ = "MPU-9250"; return true;
        case 0x73: name_ = "MPU-9255"; return true;
        default: break;
      }
    }
    // ICM-20948: select bank 0, then WHO_AM_I at 0x00 reads 0xEA.
    kind_ = Kind::ICM;
    if (writeReg(ICM_REG_BANK_SEL, 0x00) && readRegs(ICM_WHO_AM_I, &id, 1) && id == 0xEA) {
      name_ = "ICM-20948";
      return true;
    }
  }
  kind_ = Kind::NONE;
  addr_ = 0;
  return false;
}

// ---------------------------------------------------------- configuration ---
bool Gyro::configureMpu() {
  if (!writeReg(MPU_PWR_MGMT_1, 0x80)) return false;     // device reset
  delay(100);
  bool ok = writeReg(MPU_PWR_MGMT_1, 0x01);              // wake, clock = gyro PLL
  ok &= writeReg(MPU_PWR_MGMT_2, 0x00);                  // all axes on
  ok &= writeReg(MPU_SMPLRT_DIV, 0x04);                  // 1 kHz / 5 = 200 Hz
  ok &= writeReg(MPU_CONFIG,     0x03);                  // gyro low-pass ~41 Hz
  ok &= writeReg(MPU_GYRO_CONFIG, 0x08);                 // +/-500 dps, low-pass enabled
  delay(100);
  return ok;
}

bool Gyro::configureIcm() {
  bool ok = writeReg(ICM_REG_BANK_SEL, 0x00);
  ok &= writeReg(ICM_PWR_MGMT_1, 0x80);                  // device reset
  delay(100);
  ok &= writeReg(ICM_REG_BANK_SEL, 0x00);
  ok &= writeReg(ICM_PWR_MGMT_1, 0x01);                  // wake, auto clock
  ok &= writeReg(ICM_PWR_MGMT_2, 0x00);                  // gyro + accel on
  delay(50);
  ok &= writeReg(ICM_REG_BANK_SEL, 0x20);                // bank 2
  ok &= writeReg(ICM_B2_GYRO_SMPLRT_DIV, 0x04);          // 1.1 kHz / 5 = ~225 Hz
  ok &= writeReg(ICM_B2_GYRO_CONFIG_1,  0x1B);           // DLPF 51 Hz, +/-500 dps, DLPF on
  ok &= writeReg(ICM_REG_BANK_SEL, 0x00);                // back to bank 0 for reads
  delay(100);
  return ok;
}

bool Gyro::begin() {
  if (!detect()) {
    Serial.println("Gyro: no MPU-6050/6500/9250 or ICM-20948 found at 0x68/0x69 "
                   "(check wiring, 3.3 V, SDA/SCL)");
    return false;
  }
  const bool ok = (kind_ == Kind::ICM) ? configureIcm() : configureMpu();
  if (!ok) {
    Serial.printf("Gyro: %s found at 0x%02X but configuration failed\n", name_, addr_);
    kind_ = Kind::NONE;
    return false;
  }
  Serial.printf("Gyro: %s at 0x%02X\n", name_, addr_);
  return calibrate(GYRO_CAL_MS);
}

// Measure the zero-rate offset. Retries if the robot was being moved.
bool Gyro::calibrate(uint16_t ms) {
  if (kind_ == Kind::NONE) return false;
  for (int attempt = 0; attempt < 3; attempt++) {
    double sum = 0, sumSq = 0;
    int n = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
      int16_t raw;
      if (readRaw(raw)) { sum += raw; sumSq += (double)raw * raw; n++; }
      delay(4);
    }
    if (n < (int)(ms / 4) / 2) continue;                 // too many failed reads
    const double mean = sum / n;
    const double var  = sumSq / n - mean * mean;
    if (var < 400.0) {                                   // std-dev < ~0.3 dps: robot is still
      bias_    = (float)mean;
      heading_ = 0;
      rate_    = 0;
      lastUs_  = micros();
      return true;
    }
    Serial.println("Gyro calibration: robot moved, retrying...");
  }
  Serial.println("Gyro calibration failed - keep the robot still");
  return false;
}

void Gyro::update() {
  if (kind_ == Kind::NONE) return;                       // no gyro fitted (bench mode)
  const uint32_t nowUs = micros();
  if (nowUs - lastUs_ < POLL_US) return;
  float dt = (nowUs - lastUs_) * 1e-6f;
  lastUs_ = nowUs;
  if (dt > 0.25f) dt = 0.25f;                            // after a long pause

  int16_t raw;
  if (!readRaw(raw)) return;

  float r = GYRO_SIGN * ((float)raw - bias_) / LSB_PER_DPS;
  if (fabsf(r) < GYRO_DEADBAND_DPS) r = 0;
  rate_     = r;
  heading_ += r * dt;
}
