#pragma once
// ============================================================================
//  Micromouse - configuration
//  Everything you are likely to tune lives in this file.
//  Distances: millimetres | Angles: degrees | PWM: 0-255
//  See docs/TUNING.md for the order in which to tune things.
// ============================================================================
#include <stdint.h>

#define DEBUG_SERIAL 1   // 1 = print status over Serial @115200 (10 Hz)

// ---------------------------------------------------------------- Pins ------
// ESP32 DevKit. See hardware/WIRING.md for the full wiring table.
constexpr int PIN_SDA          = 21;
constexpr int PIN_SCL          = 22;

constexpr int PIN_XSHUT_LEFT   = 25;   // VL53L0X shutdown pins (used to give each
constexpr int PIN_XSHUT_FRONT  = 26;   // sensor its own I2C address at boot)
constexpr int PIN_XSHUT_RIGHT  = 27;

// L298N - left motor
constexpr int PIN_ENA = 14;            // PWM
constexpr int PIN_IN1 = 32;
constexpr int PIN_IN2 = 33;
// L298N - right motor
constexpr int PIN_ENB = 13;            // PWM
constexpr int PIN_IN3 = 18;
constexpr int PIN_IN4 = 19;

constexpr int PIN_IR    = 34;          // floor IR sensor (input-only ADC pin)
constexpr int PIN_START = 0;           // BOOT button on most DevKits (active LOW)
constexpr int PIN_LED   = 2;           // on-board LED

// Flip these if a wheel spins the wrong way
constexpr bool MOTOR_LEFT_INVERT  = false;
constexpr bool MOTOR_RIGHT_INVERT = false;
// Scale a side if the robot drifts on a straight (1.0 = no change)
constexpr float MOTOR_TRIM_LEFT   = 1.00f;
constexpr float MOTOR_TRIM_RIGHT  = 1.00f;
// Gyro: +1 if turning LEFT (counter-clockwise from above) gives a positive
// rate, -1 if your MPU-6050 is mounted so that it comes out negative.
constexpr int GYRO_SIGN = 1;

// -------------------------------------------------------------- Timing ------
constexpr uint32_t LOOP_MS         = 10;       // control loop period
constexpr uint32_t START_DELAY_MS  = 3000;     // time to take your hand away
constexpr uint32_t MAX_RUN_MS      = 120000;   // hard stop if end-zone never seen
constexpr uint32_t SENSOR_STALE_MS = 250;      // stop if a ToF goes silent this long

// ----------------------------------------------------------------- ToF ------
constexpr int      TOF_MAX_MM       = 1000;    // readings beyond this = "far"
constexpr uint32_t TOF_BUDGET_US    = 20000;   // 20 ms -> ~50 Hz per sensor
constexpr float    TOF_ALPHA_SIDE   = 0.45f;   // low-pass filter (1 = no filter)
constexpr float    TOF_ALPHA_FRONT  = 0.60f;

// ------------------------------------------------------ Maze geometry -------
// MEASURE YOUR MAZE and adjust. Defaults assume ~200 mm corridors.
constexpr int SIDE_OPEN_MM      = 150;   // side reading above this = opening
constexpr int FRONT_BLOCKED_MM  = 70;    // front reading below this = wall ahead
constexpr int FRONT_SLOW_MM     = 220;   // start slowing down below this
constexpr int TARGET_SIDE_MM    = 75;    // desired distance to the single wall followed

// Behaviour around junctions
constexpr uint8_t  OPEN_DEBOUNCE      = 3;     // consecutive loops a side must read open
constexpr uint32_t ADVANCE_MS         = 250;   // drive on after an opening so the
                                               // wheels (not the sensor) reach it
constexpr uint32_t TURN_COOLDOWN_MS   = 500;   // ignore new left-openings after a turn
constexpr uint32_t BRAKE_MS           = 60;    // pause before a turn to kill momentum
constexpr uint32_t TURN_TIMEOUT_MS    = 1800;  // give up on a turn after this long

// -------------------------------------------------------------- Speeds ------
constexpr float BASE_SPEED       = 170;   // straight-line speed
constexpr float ADVANCE_SPEED    = 140;
constexpr float TURN_SPEED       = 150;   // max in-place turn speed
constexpr float MIN_TURN_PWM     = 95;    // enough to overcome stiction
constexpr float MIN_SPEED_SCALE  = 0.55f; // slowest fraction when approaching a wall
constexpr int   MOTOR_MIN_PWM    = 60;    // L298N/motor dead-band: 0 -> off, >0 -> >= this

// ------------------------------------------------------------ Control -------
constexpr float KP_WALL     = 1.10f;   // PWM per mm of centring error
constexpr float KD_WALL     = 0.08f;   // PWM per (mm/s)
constexpr float KP_HEADING  = 2.50f;   // PWM per degree (used when no wall to follow)
constexpr float MAX_CORR    = 70.0f;   // steering clamp (PWM)
constexpr float KP_TURN     = 3.00f;   // PWM per degree of turn error

constexpr float TURN_TOL_DEG       = 4.0f;    // "close enough" for a turn
constexpr float TURN_RATE_TOL_DPS  = 25.0f;   // ...and rotating slower than this

// ------------------------------------------------------------ Gyro ----------
constexpr float GYRO_DEADBAND_DPS = 0.15f;    // ignore noise below this when integrating

// ------------------------------------------------------- End-zone (IR) ------
// The floor reflectance is sampled at power-on; the end-zone is detected when
// the reading differs from that baseline by IR_DELTA, so it works whether the
// end-zone is darker or lighter than the floor.
constexpr bool    IR_ENABLED  = true;
constexpr bool    IR_ANALOG   = true;    // false if your IR module only has a digital pin
constexpr int     IR_DELTA    = 600;     // ADC counts (0-4095)
constexpr uint8_t IR_DEBOUNCE = 4;       // consecutive loops
