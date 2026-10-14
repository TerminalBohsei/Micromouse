#pragma once
#include <stdint.h>

// Z-axis (yaw) gyro. Auto-detects the chip on the I2C bus:
//   MPU-6050 / MPU-6500 / MPU-9250 / MPU-9255   (address 0x68 or 0x69)
//   ICM-20948                                   (address 0x68 or 0x69)
// Heading is in degrees, positive = turning LEFT (counter-clockwise from above).
class Gyro {
 public:
  bool  begin();                              // detect + configure + calibrate (robot STILL)
  bool  calibrate(uint16_t ms = 800);         // re-measure the zero-rate bias (robot STILL)
  void  update();                             // integrate; call as often as possible
  float heading() const { return heading_; }
  float rate() const    { return rate_; }
  void  resetHeading()  { heading_ = 0; }
  const char* name() const { return name_; }

 private:
  enum class Kind : uint8_t { NONE, MPU, ICM };

  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n);
  bool readRaw(int16_t& raw);
  bool detect();
  bool configureMpu();
  bool configureIcm();

  Kind        kind_    = Kind::NONE;
  uint8_t     addr_    = 0;
  const char* name_    = "none";
  float       bias_    = 0;      // raw LSB at rest
  float       heading_ = 0;
  float       rate_    = 0;
  uint32_t    lastUs_  = 0;
};
