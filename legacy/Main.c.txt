/*
  USYD Micromouse - left-wall follower
  Board : ESP32 (Arduino core 3.x)
  Libs  : Adafruit VL53L0X, Adafruit MPU6050 (+ Adafruit Unified Sensor)

  Priority each step:
    1. Left open  -> turn left
    2. Front open -> go straight (centre between walls)
    3. Right open -> turn right
    4. Dead end   -> U-turn (180)
  Floor colour change (IR sensor) = end-zone -> stop.

  NOTE: ToF sensors and the MPU-6050 are I2C devices. They all share ONE
  SDA/SCL pair. The three ToFs have the same default address, so each has its
  own XSHUT pin and is given a unique address at boot.
*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>

// ---------------- PINS (change to match your wiring) ----------------
#define SDA_PIN   21
#define SCL_PIN   22

#define XSHUT_L   25
#define XSHUT_M   26
#define XSHUT_R   27

// L298N
#define IN1 16   // left motor
#define IN2 17
#define IN3 18   // right motor
#define IN4 19
#define ENA 4    // left PWM
#define ENB 5    // right PWM

#define IR_PIN 34            // floor colour sensor (input-only pin)
#define IR_END_ZONE_LEVEL LOW  // level read over the end-zone colour

// ---------------- TUNING ----------------
const int MAX_MM       = 2000;  // value used when a ToF sees nothing
const int WALL_MM      = 120;   // closer than this = wall present
const int OPEN_MM      = 150;   // farther than this = opening
const int FRONT_MM     = 70;    // front wall -> must stop/turn
const int TARGET_MM    = 60;    // distance to keep from a single wall

const int BASE_SPEED   = 150;   // 0-255
const int TURN_SPEED   = 140;
const float KP         = 0.8;   // wall-centering gain

const int ADVANCE_MS   = 180;   // drive into junction so body clears the corner
const int CLEAR_MS     = 350;   // drive after a turn to enter the new corridor
const int OPEN_CONFIRM = 2;     // consecutive readings needed to trust an opening

const float GYRO_SIGN  = 1.0;   // set to -1.0 if left turns go the wrong way
const float TURN_TOL   = 3.0;   // degrees

// ---------------- GLOBALS ----------------
Adafruit_VL53L0X tofL, tofM, tofR;
Adafruit_MPU6050 mpu;

int dL = MAX_MM, dM = MAX_MM, dR = MAX_MM;

float yaw = 0, gyroBias = 0;
unsigned long lastGyroUs = 0;
int leftOpenCount = 0;

// ---------------- MOTORS ----------------
void setMotors(int left, int right) {  // -255..255
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(IN1, left  > 0); digitalWrite(IN2, left  < 0);
  digitalWrite(IN3, right > 0); digitalWrite(IN4, right < 0);
  ledcWrite(ENA, abs(left));
  ledcWrite(ENB, abs(right));
}

void stopMotors() { setMotors(0, 0); }

// ---------------- SENSORS ----------------
bool initToF(Adafruit_VL53L0X &t, int xshut, uint8_t addr) {
  digitalWrite(xshut, HIGH);
  delay(10);
  if (!t.begin(addr)) return false;
  t.startRangeContinuous();
  return true;
}

void readToF() {
  uint16_t r;
  if (tofL.isRangeComplete()) { r = tofL.readRange(); dL = (r > MAX_MM) ? MAX_MM : r; }
  if (tofM.isRangeComplete()) { r = tofM.readRange(); dM = (r > MAX_MM) ? MAX_MM : r; }
  if (tofR.isRangeComplete()) { r = tofR.readRange(); dR = (r > MAX_MM) ? MAX_MM : r; }
}

void updateGyro() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  unsigned long now = micros();
  float dt = (now - lastGyroUs) / 1e6;
  lastGyroUs = now;
  float gz = (g.gyro.z - gyroBias) * 57.29578;  // rad/s -> deg/s
  if (fabs(gz) < 0.4) gz = 0;                   // dead-band against drift
  yaw += GYRO_SIGN * gz * dt;
}

void calibrateGyro() {
  float sum = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z;
    delay(3);
  }
  gyroBias = sum / N;
  yaw = 0;
  lastGyroUs = micros();
}

bool atEndZone() { return digitalRead(IR_PIN) == IR_END_ZONE_LEVEL; }

// ---------------- MOVES ----------------
// Positive deg = left (counter-clockwise), negative = right.
void turnBy(float deg) {
  stopMotors();
  delay(60);
  updateGyro();
  float target = yaw + deg;
  unsigned long start = millis();

  while (fabs(target - yaw) > TURN_TOL && millis() - start < 2500) {
    updateGyro();
    float err = target - yaw;
    int spd = (fabs(err) < 25) ? TURN_SPEED - 40 : TURN_SPEED;  // slow near target
    if (err > 0) setMotors(-spd,  spd);   // spin left
    else         setMotors( spd, -spd);   // spin right
  }
  stopMotors();
  delay(60);
}

// Follow walls for a single control step.
void followWalls() {
  bool wl = dL < WALL_MM;
  bool wr = dR < WALL_MM;
  int correction = 0;

  if (wl && wr)  correction = KP * (dL - dR) / 2;   // centre in corridor
  else if (wl)   correction = KP * (dL - TARGET_MM);// hug left wall
  else if (wr)   correction = KP * (TARGET_MM - dR);// hug right wall
  // else no walls: go straight

  correction = constrain(correction, -60, 60);
  // dL too big -> steer left -> slow left wheel, speed up right wheel
  setMotors(BASE_SPEED - correction, BASE_SPEED + correction);
}

// Drive straight for ms, bail out if a wall appears ahead or end-zone is hit.
void driveForwardMs(int ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    readToF();
    if (atEndZone() || dM < FRONT_MM) break;
    followWalls();
  }
  stopMotors();
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);
  stopMotors();

  pinMode(IR_PIN, INPUT);

  // Hold all ToFs in reset, then bring them up one at a time
  pinMode(XSHUT_L, OUTPUT); pinMode(XSHUT_M, OUTPUT); pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_L, LOW); digitalWrite(XSHUT_M, LOW); digitalWrite(XSHUT_R, LOW);
  delay(20);

  bool ok = true;
  ok &= initToF(tofL, XSHUT_L, 0x30);
  ok &= initToF(tofM, XSHUT_M, 0x31);
  ok &= initToF(tofR, XSHUT_R, 0x32);
  if (!ok) { Serial.println("ToF init failed"); while (1) delay(1000); }

  if (!mpu.begin()) { Serial.println("MPU6050 init failed"); while (1) delay(1000); }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("Keep robot still - calibrating gyro...");
  delay(500);
  calibrateGyro();
  Serial.println("Ready");
  delay(1500);  // time to place the robot / step back
}

void loop() {
  readToF();
  updateGyro();

  // --- Goal ---
  if (atEndZone()) {
    stopMotors();
    Serial.println("END ZONE REACHED");
    while (1) delay(1000);
  }

  // --- Debounce the "left is open" reading ---
  if (dL > OPEN_MM) leftOpenCount++; else leftOpenCount = 0;
  bool leftOpen   = leftOpenCount >= OPEN_CONFIRM;
  bool frontBlock = dM < FRONT_MM;
  bool rightOpen  = dR > OPEN_MM;

  if (leftOpen) {
    // Case 1: left turn or T/4-way junction -> always take the left
    driveForwardMs(ADVANCE_MS);
    turnBy(90);
    driveForwardMs(CLEAR_MS);
    leftOpenCount = 0;
  }
  else if (!frontBlock) {
    // Case 2: corridor/straight (or right-only side opening: ignore, keep going)
    followWalls();
  }
  else if (rightOpen) {
    // Case 3: wall ahead, left blocked, right open -> turn right
    turnBy(-90);
    driveForwardMs(CLEAR_MS);
  }
  else {
    // Case 4: dead end (front, left and right all blocked) -> U-turn
    turnBy(180);
    driveForwardMs(CLEAR_MS);
  }

  delay(5);
}
