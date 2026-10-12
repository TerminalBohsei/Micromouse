# Micromouse

A self-guiding maze robot. An ESP32 reads three Time-of-Flight (ToF) sensors
and a gyroscope, follows the **left-hand wall**, and stops on the coloured
end-zone. **Goal:** reach the end-zone as fast as possible, in a single
attempt, without touching the walls (contact = time penalty).

| Rule | Limit |
|---|---|
| Attempts | 1 |
| Robot footprint | 15 x 15 cm, max height 100 cm |
| Maze | 2 x 2 m |
| Detachable parts / projectiles | not allowed |

## Hardware

1x ESP32, 2x DC motors, 1x L298N, 3x VL53L0X ToF (left / front / right),
1x IR floor sensor, 1x MPU-6050, battery, 3D-printed chassis.
Wiring table and power notes: [`hardware/WIRING.md`](hardware/WIRING.md).

## Install

1. Install the Arduino IDE and the **ESP32** board package (core 2.x or 3.x).
2. Library Manager -> install **VL53L0X** by Pololu.
3. Open `Micromouse.ino`, select **ESP32 Dev Module**, upload.

(All `.h` / `.cpp` files sit next to `Micromouse.ino` because the Arduino IDE
only compiles sources from the sketch folder.)

## Project layout

| File | Purpose |
|---|---|
| `Micromouse.ino` | `setup()` / `loop()` |
| `Config.h` | **All pins and tunables** - start here |
| `Navigator.*` | State machine: start, follow, advance, turn, finish, fault |
| `Decision.h` | Pure left-hand-rule logic (unit-tested, no Arduino code) |
| `Sensors.*` | VL53L0X setup (unique addresses), non-blocking reads, filtering |
| `Gyro.*` | MPU-6050 Z-rate, bias calibration, heading integration |
| `Motors.*` | L298N driver with dead-band compensation, trim, braking |
| `docs/TUNING.md` | Step-by-step tuning guide |
| `tests/` | Host-side tests for the decision logic |

## How it works

- **Wall centring:** PD controller on left/right distance. Uses both walls when
  present, one wall when only one is, and the gyro heading when neither is.
- **Left-hand rule:** left open -> turn left; else front clear -> straight;
  else right open -> turn right; else turn around.
- **Junctions:** a left opening must be seen for 3 consecutive loops, then the
  robot drives on `ADVANCE_MS` so its wheels reach the opening before turning.
- **Turns:** closed-loop on the gyro, snapped to multiples of 90 degrees so
  errors never accumulate.
- **Speed:** slows automatically as a wall approaches to avoid contact.
- **End-zone:** floor IR reflectance is sampled at power-on; a large change
  stops the robot (works for darker or lighter end-zones).
- **Safety:** stops if a ToF sensor goes silent, and after `MAX_RUN_MS`.

## Running

1. Place the robot **still** and aligned with the maze walls at the start.
2. Power on; the LED blinks slowly once the gyro is calibrated.
3. Press the start button (BOOT), step away. It starts after 3 s.
4. LED fast-blinking right after boot = a sensor failed (see Serial Monitor).

Because you only get one attempt, do your tuning runs first
(see [`docs/TUNING.md`](docs/TUNING.md)).

## Tests

```
g++ -std=c++11 -Wall -I. tests/test_decision.cpp -o test_decision && ./test_decision
```
