#pragma once
// ============================================================================
//  Micromouse - configuration
//  Everything you are likely to tune lives in this file.
//  Distances: millimetres | Angles: degrees | PWM commands: 0-255
//  Order to work through: docs/TUNING.md
// ============================================================================
#include <stdint.h>

#define DEBUG_SERIAL        1   // 1 = print status over Serial @115200 (10 Hz)
#define TOF_ENABLE_VL53L1X  1   // 1 = also accept VL53L1X boards (needs the Pololu
                                //     "VL53L1X" library). Set 0 if you only have
                                //     the VL53L0X library installed.

// ===========================================================================
//  1. MEASURE YOUR ROBOT AND MAZE  (the rest of the geometry is derived)
// ===========================================================================
// Clear width of a corridor: inside face of one wall to inside face of the other.
constexpr int CORRIDOR_MM = 200;
// Distance from the face of the LEFT ToF sensor to the face of the RIGHT one.
constexpr int SENSOR_SPAN_MM = 110;
// How far the side sensors sit IN FRONT of the wheel axle (negative = behind).
constexpr int SENSOR_AHEAD_OF_AXLE_MM = 40;
// Distance from the middle of the wheel axle to the FARTHEST point of the robot
// body (usually a rear corner). The robot spins on the spot at junctions and
// dead ends, so this circle has to fit in the corridor with room to spare.
constexpr int ROBOT_RADIUS_MM = 80;
// Straight-line speed of the robot at BASE_SPEED, in mm per second. Measure it:
// type  d  in the Serial Monitor, then measure how far the robot travelled in 1 s.
constexpr int CRUISE_MM_S = 220;

// ---- derived (do not edit unless you know why) ----------------------------
// Wall distance wanted when only ONE wall is visible = robot centred.
constexpr int TARGET_SIDE_MM = (CORRIDOR_MM - SENSOR_SPAN_MM) / 2;
// A side reading above this is an opening. Walls read at most CORRIDOR-SPAN
// (robot hard against the other wall); the shortest opening reads ~CORRIDOR.
constexpr int SIDE_OPEN_MM   = TARGET_SIDE_MM + CORRIDOR_MM / 2;
static_assert(TARGET_SIDE_MM >= 30,
              "Side sensors are closer than 30 mm to the wall when centred: the "
              "VL53L0X cannot measure that reliably. Mount the sensors further "
              "from the robot's edge (smaller SENSOR_SPAN_MM is not the fix).");
static_assert(2 * ROBOT_RADIUS_MM + 10 <= CORRIDOR_MM,
              "The robot cannot spin on the spot in this corridor (needs ~10 mm spare). "
              "Make the body shorter/narrower, or move the wheels to the middle of the body.");
static_assert(SIDE_OPEN_MM > (CORRIDOR_MM - SENSOR_SPAN_MM) + 20,
              "SIDE_OPEN_MM too close to the largest wall reading - check geometry");

// ===========================================================================
//  2. PINS  (ESP32 DevKit with WROVER-IE: GPIO16/17 are PSRAM, 6-11 are flash)
//     Full wiring table: docs/WIRING.md
// ===========================================================================
constexpr int PIN_SDA          = 21;
constexpr int PIN_SCL          = 22;

constexpr int PIN_XSHUT_LEFT   = 25;   // VL53 shutdown pins (used to give each
constexpr int PIN_XSHUT_FRONT  = 26;   // sensor its own I2C address at boot)
constexpr int PIN_XSHUT_RIGHT  = 27;

// L298N - left motor
constexpr int PIN_ENA = 14;            // PWM (remove the ENA jumper cap!)
constexpr int PIN_IN1 = 32;
constexpr int PIN_IN2 = 33;
// L298N - right motor
constexpr int PIN_ENB = 13;            // PWM (remove the ENB jumper cap!)
constexpr int PIN_IN3 = 18;
constexpr int PIN_IN4 = 19;

constexpr int PIN_IR    = 34;          // floor IR sensor OUT (input-only pin)
constexpr int PIN_START = 0;           // BOOT button (active LOW)
constexpr int PIN_LED   = 2;           // GPIO2 (the DevKitC has no LED: add one if you want it)

// ULN2003 board for the 28BYJ-48 stepper (only used if STEPPER_ENABLED)
constexpr int PIN_STEP_IN1 = 4;
constexpr int PIN_STEP_IN2 = 5;
constexpr int PIN_STEP_IN3 = 15;
constexpr int PIN_STEP_IN4 = 23;

// ===========================================================================
//  3. MOTORS
// ===========================================================================
// Flip these if a wheel spins the wrong way (check with the  m  command).
constexpr bool MOTOR_LEFT_INVERT  = false;
constexpr bool MOTOR_RIGHT_INVERT = false;
// Scale a side if the robot drifts on a straight (1.0 = no change).
constexpr float MOTOR_TRIM_LEFT   = 1.00f;
constexpr float MOTOR_TRIM_RIGHT  = 1.00f;
// Dead-band: any non-zero command is lifted to at least this PWM duty.
constexpr int   MOTOR_MIN_PWM     = 60;

// Gyro: +1 if turning LEFT (counter-clockwise from above) gives a positive
// rate (board flat, component side up). -1 if you mounted it the other way.
constexpr int GYRO_SIGN = 1;

// ===========================================================================
//  4. TIMING AND SAFETY
// ===========================================================================
constexpr uint32_t LOOP_MS         = 10;       // control loop period
constexpr uint32_t START_DELAY_MS  = 3000;     // time to take your hand away
#ifndef MM_MAX_RUN_MS
#define MM_MAX_RUN_MS 300000                    // (the simulator overrides this)
#endif
constexpr uint32_t MAX_RUN_MS      = MM_MAX_RUN_MS;   // hard stop if the end-zone is never seen
constexpr uint32_t SENSOR_STALE_MS = 250;      // stop if a ToF goes silent this long

// ---------------------------------------------------------------- ToF -------
constexpr int      TOF_MAX_MM       = 1000;    // readings beyond this = "far"
constexpr uint32_t TOF_BUDGET_US    = 20000;   // 20 ms -> ~50 Hz per sensor
constexpr float    TOF_ALPHA_SIDE   = 0.60f;   // low-pass after the median filter
constexpr float    TOF_ALPHA_FRONT  = 0.70f;   // (1 = no low-pass)

// ===========================================================================
//  5. DRIVING
// ===========================================================================
constexpr int FRONT_BLOCKED_MM = 70;    // front reading below this = wall ahead, turn
constexpr int FRONT_SLOW_MM    = 220;   // start slowing down below this

constexpr float BASE_SPEED       = 150;   // straight-line speed (command 0-255)
constexpr float MIN_SPEED_SCALE  = 0.55f; // slowest fraction when approaching a wall
constexpr float TURN_SPEED       = 150;   // max in-place turn speed
constexpr float MIN_TURN_PWM     = 95;    // enough to overcome stiction

// Wall following: steering u = KP_WALL*err + KH_WALL*(heading error).
// KH_WALL damps the weaving using the gyro (much cleaner than differentiating ToF).
constexpr float KP_WALL     = 1.00f;   // PWM per mm of centring error
constexpr float KH_WALL     = 1.20f;   // PWM per degree of heading error
constexpr float KP_HEADING  = 2.50f;   // PWM per degree (gyro-only driving, no wall)
constexpr float MAX_CORR    = 60.0f;   // steering clamp (PWM)
constexpr float KP_TURN     = 3.00f;   // PWM per degree of turn error
constexpr float TURN_TOL_DEG       = 4.0f;    // "close enough" for a turn
constexpr float TURN_RATE_TOL_DPS  = 25.0f;   // ...and rotating slower than this

// ---- Junctions ------------------------------------------------------------
// A left opening must read open this long before it is trusted.
constexpr uint32_t OPEN_CONFIRM_MS    = 40;
// Time from "the sensor passed the wall's end" to "the robot reacts": ToF
// update + median + low-pass + OPEN_CONFIRM_MS. The robot keeps moving meanwhile,
// so that distance is subtracted from the advance.
constexpr uint32_t SENSE_LATENCY_MS   = 90;
// Drive on after an opening so the WHEELS (not the sensor) reach it before
// turning: sensor-to-axle offset + half the opening, minus the latency distance.
constexpr int ADVANCE_MM = SENSOR_AHEAD_OF_AXLE_MM + CORRIDOR_MM / 2
                           - (CRUISE_MM_S * (int)SENSE_LATENCY_MS) / 1000;
// If a wall is visible straight ahead, measure the advance with the front ToF
// (immune to battery sag). Otherwise, or if it misbehaves, use a timer.
constexpr bool ADVANCE_USE_FRONT_TOF = true;
constexpr uint32_t ADVANCE_TIMEOUT_PCT = 160;   // timer cap, % of the nominal time

constexpr uint32_t TURN_COOLDOWN_MS   = 500;   // ignore new left-openings after a turn
constexpr uint32_t POST_TURN_MS       = 100;   // stand still after a turn while the ToF
                                               // filters refill with fresh readings
constexpr uint32_t BRAKE_MS           = 60;    // pause before a turn to kill momentum
constexpr uint32_t TURN_TIMEOUT_MS    = 1800;  // give up on a turn after this long

// ===========================================================================
//  6. GYRO
// ===========================================================================
constexpr float GYRO_DEADBAND_DPS = 0.15f;    // ignore noise below this when integrating
constexpr uint16_t GYRO_CAL_MS    = 800;      // bias calibration time (robot STILL)

// ===========================================================================
//  7. END-ZONE (floor IR sensor)
// ===========================================================================
// The KY-033 (and IR-08H) modules have a DIGITAL output (a comparator with a
// trim-pot), so this is digital by default. The floor level is sampled when
// the run starts and the finish is declared when the output stays different
// for IR_CONFIRM_MS. Works for black->white and white->black alike.
constexpr bool     IR_ENABLED       = true;
constexpr bool     IR_ANALOG        = false;   // true only for a module with an analog pin
constexpr int      IR_FINISH_LEVEL  = -1;      // -1 = auto (change from the floor); 0/1 = fixed
constexpr int      IR_DELTA         = 600;     // ADC counts, analog mode only
constexpr uint32_t IR_CONFIRM_MS    = 30;      // must stay different this long

// ===========================================================================
//  8. STEPPER (28BYJ-48 via ULN2003) - optional "finish" action
// ===========================================================================
constexpr bool     STEPPER_ENABLED     = false;
constexpr long     STEPPER_FINISH_STEPS = 1024;  // half-steps on finish (4096 = one output turn)
constexpr uint32_t STEPPER_STEP_US     = 2000;   // time per half-step (>= 1500 or it stalls)
