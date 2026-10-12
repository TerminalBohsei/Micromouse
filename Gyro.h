#pragma once
#include <stdint.h>

// MPU-6050, Z-axis gyro only. Heading is in degrees, positive = turning LEFT.
class Gyro {
 public:
  bool  begin();                  // calibrates bias: robot must be STILL
  void  update();                 // integrate; call as often as possible
  float heading() const { return heading_; }
  float rate() const    { return rate_; }
  void  resetHeading()  { heading_ = 0; }

 private:
  bool readRaw(int16_t& raw);
  bool writeReg(uint8_t reg, uint8_t val);

  float    bias_   = 0;
  float    heading_ = 0;
  float    rate_   = 0;
  uint32_t lastUs_ = 0;
};
