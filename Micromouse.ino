// ============================================================================
//  Micromouse - self-guiding maze robot
//  ESP32 + 3x VL53L0X (ToF) + MPU-6050 (gyro) + L298N + IR floor sensor
//
//  Strategy: left-hand wall following.
//    - PD wall-centring using the left/right ToF sensors
//    - gyro heading hold when there is no wall to follow
//    - gyro-controlled 90/180 degree turns, snapped to the maze axes
//    - floor IR sensor detects the coloured end-zone and stops the robot
//
//  Libraries (Arduino Library Manager): "VL53L0X" by Pololu
//  Board: any "ESP32 Dev Module" (esp32 core 2.x or 3.x)
//  Tune everything in Config.h - see docs/TUNING.md.
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "Sensors.h"
#include "Gyro.h"
#include "Motors.h"
#include "Navigator.h"

Sensors   sensors;
Gyro      gyro;
Motors    motors;
Navigator nav(sensors, gyro, motors);

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
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!sensors.begin()) haltBlinking("ToF sensor setup failed - see messages above.");
  if (!gyro.begin())    haltBlinking("Gyro setup failed - see messages above.");

  nav.begin();
}

void loop() {
  nav.update();
}
