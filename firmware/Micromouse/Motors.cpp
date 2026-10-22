#include "Motors.h"
#include "Config.h"
#include "Decision.h"
#include "MotorMap.h"
#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------------------
//  L298N with the ENA and ENB jumper caps FITTED (enable is tied to 5 V, so the
//  bridge is always on). The ESP32 does NOT use ENA/ENB at all. Speed is set by
//  putting the PWM on the IN pins instead:
//     forward  : IN1 = PWM(duty), IN2 = LOW      (same for IN3 / IN4)
//     backward : IN1 = LOW,       IN2 = PWM(duty)
//     stop     : IN1 = LOW,       IN2 = LOW       (with EN high this BRAKES;
//                                                  the wheel does not coast)
//     brake    : IN1 = HIGH,      IN2 = HIGH
//  Each IN pin has its own LEDC channel, so both wheels run at the same time.
// ---------------------------------------------------------------------------
namespace {
constexpr int PWM_FREQ_HZ = 5000;
constexpr int PWM_BITS    = 8;
// One LEDC channel per IN pin (only used by ESP32 Arduino core 2.x).
constexpr int CH_IN1 = 0;
constexpr int CH_IN2 = 1;
constexpr int CH_IN3 = 2;
constexpr int CH_IN4 = 3;

void pwmAttach(int pin, int ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmDetach(int pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(pin);
#else
  ledcDetachPin(pin);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

void setMotor(int in1, int ch1, int in2, int ch2, float v, bool invert, float trim) {
  if (invert) v = -v;
  v = clampf(v * trim, -255.0f, 255.0f);

  const int duty = motorDuty(v, MOTOR_MIN_PWM);
  pwmWrite(in1, ch1, v > 0 ? duty : 0);     // forward: PWM on IN1
  pwmWrite(in2, ch2, v < 0 ? duty : 0);     // backward: PWM on IN2
}

void brakeMotor(int in1, int ch1, int in2, int ch2) {
  pwmWrite(in1, ch1, 255);
  pwmWrite(in2, ch2, 255);
}
}  // namespace

void Motors::begin() {
  // Outputs low BEFORE the PWM is attached so the wheels never twitch at boot.
  const int pins[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
  for (int p : pins) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
  pwmAttach(PIN_IN1, CH_IN1);
  pwmAttach(PIN_IN2, CH_IN2);
  pwmAttach(PIN_IN3, CH_IN3);
  pwmAttach(PIN_IN4, CH_IN4);
  stop();
}

// Both sides are set in the same call, so the two wheels always run together.
void Motors::drive(float left, float right) {
  // Whole-robot speed knob (Config.h MOTOR_SPEED_SCALE), floor 0.30.
  const float scale = MOTOR_SPEED_SCALE < 0.30f ? 0.30f : (MOTOR_SPEED_SCALE > 1.0f ? 1.0f : MOTOR_SPEED_SCALE);
  left *= scale;
  right *= scale;
  // Straight trim when both wheels go the same way; separate TURN trim when they
  // spin in opposite directions (turning on the spot).
  const bool sameWay = (left > 0 && right > 0) || (left < 0 && right < 0);
  setMotor(PIN_IN1, CH_IN1, PIN_IN2, CH_IN2, left,  MOTOR_LEFT_INVERT,  sameWay ? trimL_ : turnTrimL_);
  setMotor(PIN_IN3, CH_IN3, PIN_IN4, CH_IN4, right, MOTOR_RIGHT_INVERT, sameWay ? trimR_ : turnTrimR_);
}

void Motors::stop() { drive(0, 0); }

void Motors::brake() {
  brakeMotor(PIN_IN1, CH_IN1, PIN_IN2, CH_IN2);
  brakeMotor(PIN_IN3, CH_IN3, PIN_IN4, CH_IN4);
}

// Plain digital test: no PWM, each IN pin is simply driven HIGH or LOW.
// Left wheel forward, left backward, right forward, right backward, both forward.
void Motors::rawTest(uint32_t holdMs) {
  const int pins[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
  for (int p : pins) { pwmDetach(p); pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  struct Step { const char* name; int a, b, c, d; };
  const Step steps[] = {
    {"LEFT forward  (IN1 high, IN2 low)",  HIGH, LOW,  LOW,  LOW },
    {"LEFT backward (IN1 low, IN2 high)",  LOW,  HIGH, LOW,  LOW },
    {"RIGHT forward (IN3 high, IN4 low)",  LOW,  LOW,  HIGH, LOW },
    {"RIGHT backward(IN3 low, IN4 high)",  LOW,  LOW,  LOW,  HIGH},
    {"BOTH forward",                       HIGH, LOW,  HIGH, LOW },
  };
  for (const Step& st : steps) {
    Serial.println(st.name);
    digitalWrite(PIN_IN1, st.a); digitalWrite(PIN_IN2, st.b);
    digitalWrite(PIN_IN3, st.c); digitalWrite(PIN_IN4, st.d);
    delay(holdMs);
    for (int p : pins) digitalWrite(p, LOW);
    delay(400);
  }
  begin();                         // hand the pins back to the PWM
}