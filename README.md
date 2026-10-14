# Micromouse

A self-guiding maze robot. An ESP32 reads three Time-of-Flight (ToF) sensors
and a gyro, follows the **left-hand wall**, and stops when a floor IR sensor
sees the end-zone. **Goal:** reach the end-zone as fast as possible, in a
single attempt, without touching the walls (contact = time penalty).

## Hardware (from the lab parts list)

| Part | Used |
|---|---|
| Controller | 1x ESP32 DevKit (WROVER-IE) |
| Drive | 2x TT geared DC motors + wheels, 1x L298N, 1x ball caster (front) |
| Walls | 3x VL53L0X *or* VL53L1X ToF (left / front / right) - auto-detected |
| Turning | 1x gyro: MPU-9250/6500 *or* ICM-20948 - auto-detected |
| End-zone | 1x floor IR sensor (KY-033), digital |
| Power | 2x 3.7 V cells in series (**not** the 9 V) |
| Optional | 1x 28BYJ-48 + ULN2003 (see `STEPPER_ENABLED`) |

Wiring and power: [`docs/WIRING.md`](docs/WIRING.md).

## Install

**PlatformIO (VS Code):** open this folder, press Build/Upload. Libraries
install themselves (`platformio.ini`).

**Arduino IDE:** install the ESP32 board package (core 2.x or 3.x) and, from
the Library Manager, **VL53L0X** and **VL53L1X** (both by Pololu). Open
`firmware/Micromouse/Micromouse.ino`, board *ESP32 Dev Module*, upload. (Only
have the VL53L0X library? Set `TOF_ENABLE_VL53L1X 0` in `Config.h`.)

## Before the first run

1. Measure four things and put them in `Config.h` section 1.
2. Follow [`docs/TUNING.md`](docs/TUNING.md) from step 1.

## Project layout

| Path | Purpose |
|---|---|
| `firmware/Micromouse/Micromouse.ino` | `setup()` / `loop()` |
| `.../Config.h` | **All pins and tunables - start here** |
| `.../Navigator.*` | State machine: wait, countdown, follow, advance, turn, finish, fault; bench commands |
| `.../Decision.h` | Pure left-hand-rule logic |
| `.../Sensors.*` + `RangeFilter.h` | ToF setup (unique addresses, L0X/L1X), non-blocking reads, median filter |
| `.../Gyro.*` | MPU-6050/6500/9250 + ICM-20948 yaw gyro: detect, bias calibration, heading |
| `.../Motors.*` + `MotorMap.h` | L298N driver with dead-band compensation, trim, braking |
| `.../Floor.*` | End-zone detection (digital, compares with the floor at start) |
| `.../Stepper.*` | Optional 28BYJ-48 action on finish |
| `docs/` | `WIRING.md`, `TUNING.md` |
| `tests/` | Unit tests + closed-loop maze simulator (`make test`) |
| `legacy/Main.c.txt` | The first single-file prototype (kept for reference, not built) |

## How it works

* **Wall centring:** steering = `KP_WALL` x (centring error) + `KH_WALL` x (gyro
  heading error). Uses both walls when present, one when only one is, and the
  gyro heading when neither is.
* **Left-hand rule:** left open -> turn left; else front clear -> straight; else
  right open -> turn right; else turn around.
* **Junctions:** a left opening must read open for `OPEN_CONFIRM_MS`. The robot
  then drives on `ADVANCE_MM` (measured with the front ToF when a wall is
  visible, else by a timer) so the *wheels* are in the opening before it spins.
* **Turns:** closed-loop on the gyro, snapped to multiples of 90 degrees so
  errors never accumulate.
* **Filtering:** median-of-3 on every ToF reading, so one glitch can never fake
  a junction.
* **End-zone:** the floor IR level is sampled at the start; a change that lasts
  `IR_CONFIRM_MS` stops the robot.
* **Safety:** stops if a ToF goes silent, or after `MAX_RUN_MS`. The gyro is
  re-calibrated just before the run starts.

## Running

1. Place the robot **still** and square in the start cell.
2. Press BOOT (or type `s`). The LED blinks fast for 3 s, then the gyro
   calibrates (~1 s). Keep clear of it for ~4 s.
3. It stops on the end-zone and blinks. Press BOOT to return to idle.

## Known limits

* **Left-hand wall following only finds the end-zone if it touches a wall
  connected to the start.** If the end-zone sits inside an island (a loop of
  walls not connected to the outer wall), this strategy will circle forever.
  Check the official maze rules.
* The advance is dead-reckoned (no wheel encoders). Fresh battery for the run.
* All of this has been compiled for ESP32 and simulated, **not yet run on the
  real robot.** Expect to tune.
