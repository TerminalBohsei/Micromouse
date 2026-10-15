// ============================================================================
//  Micromouse - self-guiding maze robot
//  ESP32 + 3x VL53L0X/L1X (ToF) + L298N + 2 TT motors + floor IR sensor (+ optional 28BYJ-48 stepper)
//
//  Strategy: left-hand wall following.
//    - PD wall-centring using the left/right ToF sensors
//    - 90/180 degree turns are timed spins (calibrate TURN_90_MS / TURN_180_MS)
//    - floor IR sensor detects the end-zone and stops the robot
//
//  Libraries:  "VL53L0X" and "VL53L1X" by Pololu (see platformio.ini)
//  Board:      ESP32 Dev Module (esp32 core 2.x or 3.x)
//  Tune everything in Config.h - see docs/TUNING.md.
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Stepper.h"
#include "Navigator.h"

Sensors   sensors;
Motors    motors;
Stepper   stepper;
Navigator nav(sensors, motors, stepper);

[[noreturn]] static void haltBlinking(const char* msg) {
  Serial.println(msg);
  motors.stop();
  while (true) {                       // fast blink = hardware problem
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(80);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  motors.begin();                      // motors off before anything else
  stepper.begin();
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);                 // a stuck I2C bus must never freeze the robot

  delay(300);
  Serial.println("\n=== Micromouse ===");
  if (!sensors.begin()) haltBlinking("ToF sensor setup failed - see messages above.");
#if BENCH_FRONT_ONLY
  Serial.println("BENCH MODE: only the front ToF is used (set BENCH_FRONT_ONLY 0 for the real run).");
#endif

  nav.begin();
}

void loop() {
  nav.update();
}
