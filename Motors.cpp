#include "Motors.h"
#include "Config.h"
#include "Decision.h"
#include <Arduino.h>
#include <math.h>

namespace {
constexpr int PWM_FREQ_HZ = 2000;
constexpr int PWM_BITS    = 8;
constexpr int CH_LEFT     = 0;   // only used by ESP32 Arduino core 2.x
constexpr int CH_RIGHT    = 1;

void pwmAttach(int pin, int ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(pin, ch);
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

void setMotor(int in1, int in2, int en, int ch, float v, bool invert, float trim) {
  if (invert) v = -v;
  v = clampf(v * trim, -255.0f, 255.0f);

  if (fabsf(v) < 1.0f) {            // off (coast)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmWrite(en, ch, 0);
    return;
  }
  // Skip the dead-band: any non-zero command moves the wheel.
  const int duty = MOTOR_MIN_PWM + (int)((fabsf(v) / 255.0f) * (255 - MOTOR_MIN_PWM));
  digitalWrite(in1, v > 0 ? HIGH : LOW);
  digitalWrite(in2, v < 0 ? HIGH : LOW);
  pwmWrite(en, ch, duty);
}

void brakeMotor(int in1, int in2, int en, int ch) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  pwmWrite(en, ch, 255);
}
}  // namespace

void Motors::begin() {
  const int pins[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
  for (int p : pins) pinMode(p, OUTPUT);
  pwmAttach(PIN_ENA, CH_LEFT);
  pwmAttach(PIN_ENB, CH_RIGHT);
  stop();
}

void Motors::drive(float left, float right) {
  setMotor(PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT,  left,  MOTOR_LEFT_INVERT,  MOTOR_TRIM_LEFT);
  setMotor(PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT, right, MOTOR_RIGHT_INVERT, MOTOR_TRIM_RIGHT);
}

void Motors::stop() { drive(0, 0); }

void Motors::brake() {
  brakeMotor(PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT);
  brakeMotor(PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT);
}
