# Wiring

Pin numbers match `firmware/Micromouse/Config.h`. If you wire differently,
change them there. **Check this table against your build before powering up.**

Board: the lab's ESP32 DevKit with the WROVER-IE module. GPIO 16/17 belong to
its PSRAM and GPIO 6-11 to the flash, so none of those are used.

## Power (read this first)

```
 2x 3.7 V cells in SERIES (7.4 V, 8.4 V fresh)
   + ──────────────► L298N "12V" terminal
   - ──────────────► L298N GND ──┬──► ESP32 GND ──► every sensor GND
                                 └──► ULN2003 GND (if used)
 L298N "5V" terminal ──► ESP32 "5V" pin   (leave the L298N 5V-EN jumper ON)
 ESP32 "3V3" pin ──► ToF VIN x3, gyro VCC, floor IR VCC
```

* **Use the two 3.7 V cells in series, not the 9 V battery.** A 9 V alkaline
  collapses under two TT motors and the ESP32 will brown-out and reboot mid-run.
  The L298N also drops ~2 V, so 7.4-8.4 V in gives the motors ~5.4-6.4 V, which
  is right for TT motors. The L298N 5 V regulator needs at least ~7 V in.
* **Never plug in USB and run from the battery at the same time** unless you
  have checked your board for back-feed protection.
* Put a **470-1000 uF capacitor** across the ESP32 5 V / GND, and a **100 nF
  ceramic** across each motor's two terminals (as close to the motor as possible).
* Put a **10 kΩ pull-down from ENA to GND and from ENB to GND.** The ESP32 pins
  float for a moment at power-up and would otherwise twitch the wheels.
* **Remove the two jumper caps on ENA and ENB** (the PWM comes from the ESP32).
* Do **not** power the floor IR module from 5 V: its OUT pin would then output
  5 V into a 3.3 V ESP32 pin.

## I2C bus (3 ToF sensors + gyro share it)

| Signal | ESP32 pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |

| Device | Address | Extra pin |
|---|---|---|
| ToF left  | 0x30 (set at boot) | XSHUT -> GPIO 25 |
| ToF front | 0x31 (set at boot) | XSHUT -> GPIO 26 |
| ToF right | 0x32 (set at boot) | XSHUT -> GPIO 27 |
| Gyro (MPU-9250/6500 or ICM-20948) | 0x68 or 0x69 (auto-detected) | none |

All three ToFs power up at 0x29. The firmware holds them in reset via XSHUT
and wakes them one at a time. VL53L0X **and** VL53L1X boards are both accepted
(the lab list says "vl53l0/1x"); the boot message shows which you have.

Gyro: fit **one** (the MPU-9250/6500 *or* the ICM-20948). Leave unused pins
(INT, FSYNC, NCS, ADO...) unconnected. The Adafruit ICM-20948 board answers on
0x69 and the blue MPU board on 0x68 - both work.

## L298N motor driver

| L298N | ESP32 | Notes |
|---|---|---|
| ENA | GPIO 14 | left motor PWM (+10 kΩ to GND) |
| IN1 | GPIO 32 | left direction |
| IN2 | GPIO 33 | left direction |
| ENB | GPIO 13 | right motor PWM (+10 kΩ to GND) |
| IN3 | GPIO 18 | right direction |
| IN4 | GPIO 19 | right direction |
| GND | GND | **must be shared with the ESP32** |

## Floor IR sensor (end-zone)

| Module | ESP32 | Notes |
|---|---|---|
| OUT | GPIO 34 | input-only pin, digital |
| VCC | 3V3 | **not 5 V** |
| GND | GND | |

Use the **KY-033** (range 1-25 mm), pointing **down**, 5-10 mm above the floor,
ahead of the wheels. The IR-08H is an obstacle sensor (2-30 cm) and is the
wrong tool for the floor. Turn the blue trim-pot until the on-board LED
switches between floor and end-zone (see TUNING step 1).

## Start button, LED

| Part | ESP32 | Notes |
|---|---|---|
| Start button | GPIO 0 | the BOOT button on the board |
| LED | GPIO 2 | the DevKit has no LED: add one (+330 Ω to GND) if you want it |

## Stepper (optional, `STEPPER_ENABLED`)

| ULN2003 board | ESP32 |
|---|---|
| IN1 | GPIO 4 |
| IN2 | GPIO 5 |
| IN3 | GPIO 15 |
| IN4 | GPIO 23 |
| + / - | 5 V (L298N 5 V terminal) / GND |

The 28BYJ-48 draws ~240 mA. The firmware only moves it after the robot has
stopped on the finish line and switches the coils off afterwards, so the
L298N's small 5 V regulator is not overloaded.

## Mounting notes

* ToF sensors: left and right facing straight out sideways, front straight
  ahead, all at the same height, with nothing blocking the beam. When the robot
  is centred each side sensor should read **at least ~35-40 mm** to the wall
  (the VL53L0X is unreliable closer than ~30 mm); the firmware refuses to build
  if your measurements say otherwise.
* Gyro: mount flat, component side up, away from the motors. If turning left
  makes `hdg` go **down**, set `GYRO_SIGN = -1`.
* Keep the whole robot within the 15 x 15 cm limit.
* The wheels must be able to spin the robot on the spot inside a corridor:
  measure from the middle of the axle to the farthest corner of the body and
  put that in `ROBOT_RADIUS_MM`. Keep at least ~20 mm spare per side.
