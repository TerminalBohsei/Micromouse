#pragma once
// 2-D physics world: a differential-drive robot in a wall maze, with simulated
// VL53 ToF sensors (noise, glitches, async 50 Hz), a gyro (bias + noise) and a
// first-order motor model with a dead-band.
#include <stdint.h>
#include <vector>
#include <string>

struct Seg { double x1, y1, x2, y2; };

struct World {
  // ---- maze ----
  double P = 212.0;          // cell pitch (200 mm clear + 12 mm wall)
  double T = 12.0;           // wall thickness
  int    W = 0, H = 0;
  std::vector<Seg> segs;
  int finishCx = 0, finishCy = 0;

  // ---- robot (reference point = middle of the wheel axle) ----
  double x = 0, y = 0, th = 0;           // mm, mm, rad (0 = +x, CCW positive)
  double vl = 0, vr = 0;                 // wheel speeds mm/s
  double dutyL = 0, dutyR = 0;           // signed 0..255 from the Motors mock
  bool   braking = false;
  double wheelK[2] = {1.00, 0.96};       // wheel mismatch
  double track = 95.0;                   // wheel separation
  double bodyFront = 50, bodyRear = 55, halfW = 55;   // body relative to the axle (spin radius ~78 mm)
  double speedMax = 360;                 // mm/s at full duty

  // ---- sensors ----
  double nextSample[3] = {0, 0, 0};
  bool   ready[3] = {false, false, false};
  int    mm[3] = {0, 0, 0};
  double gyroBias = 1.5;                 // deg/s
  double omegaDps = 0;
  double glitchProb = 0.004, tofSigma = 2.5;

  // ---- bookkeeping ----
  uint64_t nowUs = 0;
  int  contacts = 0;
  bool inContact = false;
  double distance = 0;
  uint64_t rng = 12345;
  bool startPressed = false;

  double rnd();                          // uniform 0..1
  double gauss();                        // N(0,1)
  void generateMaze(int w, int h, uint64_t seed);
  void placeRobot(int cx, int cy, double dxMm, double dyMm, double yawDeg);
  void step(uint32_t us);
  double raycast(double sx, double sy, double ang, double maxd) const;
  bool irOnFinish() const;
  bool collides() const;
  void samplePose(double lx, double ly, double& wx, double& wy) const;
};

extern World* g_world;
