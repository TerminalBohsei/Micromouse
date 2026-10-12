# Tuning guide

Everything is in `Config.h`. Open the Serial Monitor at 115200 baud: it prints
`st=<state> L= F= R= hdg= u=` ten times per second.

States: 0 wait, 1 countdown, 2 follow, 3 advance, 4 turning, 5 finished, 6 fault.

Work through these in order, re-uploading after each change. Do the early
steps with the wheels off the ground.

## 1. Sanity checks (robot in your hand)

- **ToF:** wave a hand at each sensor. Left/front/right values must change for
  the matching side. If sides are swapped, swap the XSHUT pins in `Config.h`.
- **Gyro:** rotate the robot left. `hdg` must increase. If not, set
  `GYRO_SIGN = -1`.
- **IR:** the boot message prints the floor baseline. Hold the sensor over the
  end-zone colour and check the reading changes by more than `IR_DELTA`.

## 2. Motors (wheels off the ground)

- Both wheels must spin forward. Flip `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT`
  if not.
- If the robot creeps in an arc on a straight, adjust `MOTOR_TRIM_LEFT/RIGHT`
  (e.g. 0.95 on the faster side).
- Raise `MOTOR_MIN_PWM` if a wheel hums but does not move at low speed.

## 3. Maze geometry

Measure a real corridor, then set:

- `TARGET_SIDE_MM` - wall distance when only one wall is visible.
- `SIDE_OPEN_MM` - larger than the widest wall gap you expect, smaller than the
  distance across a junction.
- `FRONT_BLOCKED_MM` - how close to a wall to start turning. Increase it if the
  robot clips the wall while spinning (your robot's length matters here).

## 4. Straight-line wall following

1. Start with `BASE_SPEED` around 130.
2. Raise `KP_WALL` until it corrects quickly, stop when it starts weaving.
3. Raise `KD_WALL` to damp the weaving.
4. Raise `BASE_SPEED` in steps of 10-15; re-check steps 2-3 each time.

## 5. Turns

- Overshoots or oscillates: lower `TURN_SPEED` or `KP_TURN`.
- Stalls short of the target: raise `MIN_TURN_PWM`.
- Turns take too long: widen `TURN_TOL_DEG` slightly (wall following corrects
  small leftover angles anyway).

## 6. Junctions

- Turns left too early / clips the corner: increase `ADVANCE_MS`.
- Turns too late: decrease it.
- Random false left turns: raise `OPEN_DEBOUNCE`, or check the left sensor is
  not seeing through gaps.

## 7. Speed for the final run

Because you get one attempt, finish with a few full practice runs and leave a
safety margin: reliability beats the last 5% of speed, since every wall touch
costs time.

## Power note

A 9 V battery sags under motor load and the L298N drops about 2 V, so voltage
falls as the battery drains. Tune with a fresh battery and use a fresh one for
the real run. A regulated 5 V supply for the ESP32 (not through its 3.3 V pin)
avoids brown-out resets.
