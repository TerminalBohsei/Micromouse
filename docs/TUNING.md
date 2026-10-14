# Bring-up and tuning guide

Everything is in `firmware/Micromouse/Config.h`. Open the Serial Monitor at
115200 baud (send "Newline"). While running it prints, 10x a second:

`st=<state> L= F= R= hdg= u= ir=`

States: 0 idle, 1 countdown, 2 follow, 3 advance, 4 turning, 5 finished, 6 fault.

In the idle state you can type single-letter **bench commands**:

| Key | Does |
|---|---|
| `s` | start the run (same as the BOOT button) |
| `m` | spin LEFT wheel, RIGHT wheel, then both backwards (wheels off the ground) |
| `d` | drive straight for 1 s - measure the distance for `CRUISE_MM_S` |
| `l` `r` `u` | turn 90 left / 90 right / 180 and print the error |
| `c` | re-calibrate the gyro |
| `f` | stepper test |
| `?` | help |

Work through the steps **in order**.

## 0. Measure (once)

Put these in section 1 of `Config.h`. Everything else is derived from them.

| Constant | What to measure |
|---|---|
| `CORRIDOR_MM` | clear width between two facing maze walls |
| `SENSOR_SPAN_MM` | left ToF face to right ToF face |
| `SENSOR_AHEAD_OF_AXLE_MM` | how far the side sensors are in front of the wheel axle (negative = behind) |
| `ROBOT_RADIUS_MM` | middle of the axle to the farthest corner of the body |

At boot the robot prints the derived values and the **spin clearance**
(`CORRIDOR/2 - ROBOT_RADIUS`). Keep it **>= 20 mm**. The simulator showed that
a robot with ~11 mm of clearance clips a wall whenever it is more than ~1 cm
off-centre when it starts a turn; with 22 mm it never did, even with bad sensors.

## 1. Sanity checks (robot in your hand, motors off)

* **Boot log** lists the ToF models, the gyro name and the derived distances.
  A missing sensor stops the robot and blinks fast.
* **ToF:** wave a hand at each sensor; `L`, `F`, `R` must follow the matching
  side. If left/right are swapped, swap the XSHUT pins in `Config.h`.
  With the robot centred in a corridor `L` and `R` should both be close to
  `TARGET_SIDE_MM` (printed at boot).
* **Gyro:** rotate the robot LEFT. `hdg` must **increase**. If not,
  `GYRO_SIGN = -1`. Leave it still for 30 s: `hdg` should move < 1 degree.
* **Floor IR:** hold the sensor over the normal floor and over the end-zone
  and watch `ir=`. It must read different (0 vs 1). If not, turn the trim-pot
  until the on-board LED changes between the two. The firmware compares with the
  floor level it sees at the start, so black->white and white->black both work.

## 2. Motors (wheels off the ground) - type `m`

* Each wheel must turn so the robot goes toward its front. If not, flip
  `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT`.
* A wheel that hums but doesn't turn: raise `MOTOR_MIN_PWM`.

## 3. Speed - type `d`

Put the robot on the floor, type `d`, measure how far it went in 1 s and set
`CRUISE_MM_S`. If it veers, adjust `MOTOR_TRIM_LEFT/RIGHT` (e.g. 0.95 on the
faster side). This number sets the junction advance, so do it with a **fresh,
fully charged battery**.

## 4. Turns - type `l`, `r`, `u`

Each prints `error`. Aim for < 5 degrees, repeatably.

* Overshoots / oscillates: lower `TURN_SPEED` or `KP_TURN`.
* Stalls short of the target: raise `MIN_TURN_PWM`.
* Takes too long: widen `TURN_TOL_DEG` slightly (wall following corrects small
  leftovers; headings are also snapped to multiples of 90 degrees).

## 5. Straight-line wall following (a straight corridor)

1. Start with `BASE_SPEED` 130.
2. Raise `KP_WALL` until it corrects quickly; stop when it starts weaving.
3. Raise `KH_WALL` to damp the weaving (it uses the gyro).
4. Raise `BASE_SPEED` in steps of 10-15 and re-check 2-3.

## 6. Junctions (a T-junction and a left corner)

The robot sees a left opening, drives on `ADVANCE_MM`, then spins left. If the
turn happens too early (clips the corner) or too late (overshoots the opening):

* First check `SENSOR_AHEAD_OF_AXLE_MM` and `CRUISE_MM_S`: `ADVANCE_MM` is
  computed from them.
* Then trim `SENSE_LATENCY_MS` (larger = shorter advance).
* False left turns on straights: raise `OPEN_CONFIRM_MS`, or check the left
  sensor is not seeing through a gap or hole in the wall.

## 7. Final run

You get one attempt, so finish with several full practice runs and leave a
safety margin: every wall touch costs time. Use a **fresh battery**, place the
robot **still** and **square** in the start cell, press the button and step
away. Do not touch it for ~4 s (3 s countdown + gyro calibration).
Set `DEBUG_SERIAL 0` if you like (it costs almost nothing either way).

## The simulator

`cd tests && make test` runs the unit tests and then drives the **real**
`Navigator.cpp` through 24 random 9x9 mazes with simulated ToF noise, glitches,
gyro bias and mismatched wheels. `./sim_run --stress` makes everything worse.
`./sim_run -t 9` traces wall contacts for maze 9 (`MM_VERBOSE=1` for telemetry).

It is a logic check, **not** a guarantee: the physics are my own model, and
real motors, floors and walls will differ. Use it to catch logic bugs, then
trust the practice runs.
