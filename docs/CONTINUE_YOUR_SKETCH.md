# Continue Kohsei's sketch

Use the provided firmware/Micromouse folder. It continues the setup(),
printReading() and loop() structure and supplies the sensor/motor code that the
original sketch is missing. This is an ESP32 sketch, not an Arduino Nano sketch.

## What to fix in the original

- A bare #include is incomplete: it needs a header name.
- Every empty #define expands to nothing. Give each GPIO a number. Config.h
  uses typed constexpr constants instead and explains the motor mapping.
- motor1pin1 and motor1pin_ are different identifiers. Use one consistent name.
- delay(100) needs a semicolon. The supplied moving loop instead uses millis()
  to print periodically while continuing steering and emergency-stop checks.
- cml/cmm/cmr are unassigned variables. They need actual sensor measurements.
  The sensor drivers return millimetres; divide by 10.0 only if displaying cm.
- The stock VL53 ToFs use I2C. They do not have a trigger/echo or send/receive
  pair like some ultrasonic sensors. All three share SDA and SCL; each needs
  its own XSHUT wire so the code can assign separate addresses at boot.
- L298N IN1–IN4 select motor directions. ENA/ENB add independent PWM speed
  control. Use two TT motors for differential steering.

## Connect these signals

Motor 1 means LEFT, motor 2 means RIGHT. Numbers below are ESP32 GPIO numbers,
not header positions. The map avoids WROVER PSRAM GPIO16/17.

| Module pin | ESP32 GPIO | Replaces original name |
| --- | --- | --- |
| L298N IN1 | 26 | motor1pin1 |
| L298N IN2 | 27 | motor1pin2 |
| L298N IN3 | 18 | motor2pin1 |
| L298N IN4 | 19 | motor2pin2 |
| L298N ENA | 25 | Add: motor 1 speed |
| L298N ENB | 33 | Add: motor 2 speed |
| All ToF SDA + one gyro SDA | 21 | Shared I2C data |
| All ToF SCL + one gyro SCL | 22 | Shared I2C clock |
| Left / front / right XSHUT | 13 / 14 / 23 | Separate sensor shutdown control |
| KY-033 OUT/S | 32 | Downward white finish detector |
| Optional button to GND | 4 | Start/stop |

Remove ENA/ENB jumpers before attaching PWM. Connect all grounds together and
follow START_HERE.md for sensor supply voltages, XSHUT pull-ups and motor power.
Use 3.3 V-safe sensor logic. The ToF GPIO1/interrupt pins are unused here;
readiness is polled through I2C. Gyro interrupt and FSYNC are also unused.

## First session: sensor readings

1. Extract the complete package. Open firmware/Micromouse/Micromouse.ino in
   Arduino IDE, with every supplied .h file in the same folder.
2. Select the actual ESP32 board and USB port. Install Adafruit VL53L0X and
   VL53L1X by Pololu, including dependencies offered by Library Manager.
3. Leave motor power disconnected. Keep AllowMotorMotion and
   NavigationCalibrated false. The full sketch already prints sensor readings.
   If you want to start with the smaller version, paste FirstUpload.ino.txt into
   Micromouse.ino temporarily; it uses the same headers and does not drive motors.
   A text-only sketch pasted without those headers will not compile.
4. Upload, hold the chassis still for gyro calibration and open Serial Monitor
   at 115200 baud. Confirm the reported gyro/ToF models.
5. Put a wall successively in front, left and right. The matching distance must
   change. Test at known distances within the initial 30–1200 mm envelope.
6. Move the KY-033 over black floor, then white tape. Adjust its mounting height
   and trim potentiometer; set WhiteDigitalLevel from the measured polarity.
7. Rotate the chassis by hand: left should increase yaw, right should decrease
   it. Adjust GyroSign if necessary. Check stationary drift.

## Agree which wall to follow

The original project request specified RIGHT; Kohsei's new comment mentions
LEFT. One Config.h setting now handles either without exchanging physical
sensor wires or changing GyroSign:

```cpp
constexpr bool FollowLeftWall = false; // RIGHT
// Change false to true for LEFT.
```

| Observations | RIGHT mode | LEFT mode |
| --- | --- | --- |
| Preferred side has a confirmed opening | Turn right | Turn left |
| Preferred side closed, front clear | Forward | Forward |
| Preferred side and front closed, other side open | Turn left | Turn right |
| All three blocked | Turn 180 degrees | Turn 180 degrees |
| Confirmed white finish after black | Stop | Stop |

There are two separate controls: the table decides which passage to take;
wall-distance feedback adjusts the wheel speeds while driving along a wall.
Too close to the RIGHT wall means steering left; too close to the LEFT wall
means steering right. The gyro controls the angle of a turn, rather than hoping
a fixed motor-on delay always produces 90 degrees. A side opening is confirmed
over fresh readings and the robot advances to its calibrated turning position.

Both modes use TargetWallMm, WallPresentMm and JunctionAdvanceMs. Physical
telemetry stays L/F/R and yaw stays positive LEFT. T/Y/U serial turn tests still
mean physical right 90, left 90 and left 180, regardless of the selected wall.

## Next sessions: motors, turns, then maze

Restore the full supplied Micromouse.ino after the small sensor-only sketch.
Follow START_HERE.md in this order: motor F/B bench checks with wheels lifted;
T/Y/U turn calibration on a clear test floor; straight-wall following; one
corner; a T-junction; a dead end; then white finish detection. Enable
AllowMotorMotion only after power/direction checks, and NavigationCalibrated
only after measuring corridor/chassis clearance and approach/exit travel times.
Use S to start after the three-second delay. Q stops while connected; use the
reachable motor-power switch during untethered tests.

Choose one tested wall strategy for competition. Following a wall does not
guarantee finding an internal finish in arbitrary mazes with isolated wall
islands/loops. Gyro feedback does not measure forward distance; timing needs
rechecking with battery load and floor grip. Invalid/no-return ranges stop the
robot rather than being treated as openings. Verify these limits on your own
development course and confirm the organiser's topology rules.

## Work split

Kohsei can verify wiring, ToF identity/addressing and black/white readings;
Dava can verify motor direction, gyro turns and navigation scenarios. Review
Config.h together and record the exact settings and commit for each physical
test in TEST_LOG.md. Add the firmware folder to your shared Git branch; the
provided Arduino entry point is Micromouse.ino rather than Main.c.

References: [Adafruit multi-ToF setup](https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code),
[ST L298 driver](https://www.st.com/resource/en/datasheet/l298.pdf).
