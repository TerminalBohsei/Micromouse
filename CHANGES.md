# What changed from the `kohsei` upload, and why

## It did not build
| Problem | Fix |
|---|---|
| `Main.c` (old prototype with its own `setup()/loop()`) sat next to the `.ino`; the compiler tried to build it as C | moved to `legacy/Main.c.txt` |
| `test_decision.cpp` sat in the sketch folder with `#include "../Decision.h"` | moved to `tests/` |
| README pointed at `hardware/`, `docs/`, `tests/` folders that didn't exist (the upload was flat) | real folder layout; PlatformIO + Arduino IDE both supported |

## It didn't match the parts
| Problem | Fix |
|---|---|
| Gyro code was for an **MPU-6050**; the lab has MPU-9250/6500 and ICM-20948 | new auto-detecting gyro driver |
| IR sensor read as **analog**; KY-033 / IR-08H only have a digital output | digital floor sensor, compares with the floor level at start (analog still selectable) |
| ToF library handled only VL53L0X; the lab list says "vl53l0/1x" | auto-detects L0X or L1X per sensor |
| Power: 9 V battery | documented 2x 3.7 V in series, caps, pull-downs |
| Stepper not supported | optional non-blocking 28BYJ-48 driver (`STEPPER_ENABLED`) |

## Logic bugs found (most by the simulator)
| Problem | Fix |
|---|---|
| One ToF glitch kept "left open" true for ~6 loops, enough to fake a junction (reproduced in `test_unit.cpp`) | median-of-3 filter, time-based debounce |
| `TARGET_SIDE_MM = 75` with a 200 mm corridor leaves a ~11 cm wide robot -5 mm to the other wall | wall target, opening threshold and advance now **derived** from measured geometry, with compile-time checks |
| Advance used gyro only, so the robot arrived at a junction off-centre and clipped the wall when spinning | keeps following the remaining wall during advance |
| Advance was a fixed 250 ms (battery dependent, and ~60 mm: too short to reach the middle of the opening) | computed from geometry; measured with the front ToF when possible |
| Two left turns in a row (U-bend): the turn cooldown hid the open left, so it did a 180 instead | cooldown no longer applies when a wall is ahead |
| After a turn the filters still held readings taken mid-spin | filters restart; robot stands still `POST_TURN_MS` |
| ToF polled on every loop pass (3 I2C reads per ~ms) | polled every 4 ms; gyro read at 250 Hz; I2C timeout so a stuck bus can't freeze the robot |
| Gyro bias measured once at power-on | re-measured just before each run (temperature drift) |
| `MAX_RUN_MS` 120 s: too short for big mazes (worst simulated case 160 s) | 300 s |

## Added
* Bench commands (`m d l r u c f ?`) so wheels, speed and turns can be tuned without a maze.
* Press BOOT again after a finish/fault to re-arm without resetting.
* Host tests + closed-loop simulator (`tests/`).

## Gyro removed

The robot has no gyro, so all gyro code is gone (`Gyro.*`, `GYRO_*`, `KH_WALL`,
`BENCH_NO_GYRO`).

| Was | Now |
|---|---|
| Closed-loop 90/180 turns on the gyro heading | Timed spins: `TURN_90_MS`, `TURN_180_MS` (calibrate with `l`, `u`) |
| Steering = `KP_WALL` x error + `KH_WALL` x gyro heading error | `KP_WALL` x error + `KD_WALL` x error rate (from the ToF readings) |
| Gyro heading hold when no wall is visible | Drives straight (use `MOTOR_TRIM_*` to make "straight" true) |
| Side readings of any size were steered on | Readings beyond `STEER_WALL_MAX_MM`, e.g. a half-seen doorway, are ignored and one reading is capped at `MAX_WALL_ERR_MM`. Without this the robot veered into doorways and never recovered |
| Maze run disabled while `BENCH_NO_GYRO` was 1 | That flag is gone: the maze run works as soon as all three ToF sensors are fitted |

Simulator (24 random 9x9 mazes, noise, glitches, mismatched wheels, battery
-6% / 0 / +5%): 24/24 reach the end-zone with zero wall contacts. Stress mode
(4x glitches, 2x noise, wheel skew +-7%, battery +-10%): 23/24 reach it, 20/24
with zero contacts. Timed turns are the weak spot: a battery 6% off makes a
90-degree turn come out 83-94 degrees.
