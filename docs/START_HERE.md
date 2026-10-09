# Build and test in this order

## 1. Confirm the actual parts and course dimensions

Write down the ESP32 variant, ToF model, gyro model, motor voltage/stall current,
driver module, IR module/output voltage, and battery type/current capability.
The stock photo shows an ESP32-WROVER-IE. The ToF label VL53L0/1XV2 does not
identify one chip family; the firmware reports each detected model. Available
gyros are MPU-9250/6500 and an Adafruit ICM-20948 breakout: choose ONE, not both.
Use two TT DC motors, the L298N and a KY-033 for this build. The KY-033 is the
downward floor reflectance detector; the IR-08H is an obstacle detector and is
not its replacement. Verify the actual borrowed modules and their voltage limits.

Ask the organiser for the corridor width, finish-strip width, whether the maze has
loops/isolated wall islands, whether the finish is internal, and whether a start
button is allowed. These are course specifications, not a preview of the map.
Do not assume the standard dimensions of a different Micromouse competition.

Build your own straight corridor, 90-degree corners, T-junction, cross-junction,
dead end and white finish strip for development, if the rules permit. This does
not require seeing or practising the official maze.

## 2. Place the sensors before finalising the chassis

- Front ToF: face straight ahead, with a clear field of view.
- Left/right ToFs: point perpendicular to travel, at 90 degrees left/right,
  preferably close to the drive-axle line. Record their forward/backward offset.
- Keep all three level, at the same practical height below the 25 cm wall top.
  Test the beam against the wall, not the floor, wheels or chassis. The final
  height must follow the actual module's field of view and wall construction.
- KY-033 reflectance sensor: face DOWN near the front underside. The inventory
  lists a 1–25 mm working distance; choose a close mounting height by testing
  real black floor and white tape, then adjust its sensitivity potentiometer.
- Gyro: mount rigidly and flat, Z upwards; software assumes this yaw axis.
- Mount battery securely and low; protect cables from wheels and the floor.
- Use the two DC motors for drive and a caster for support. No stepper is needed
  by this implementation, unless the organiser requires it for another purpose.

Do not fill the entire 15 cm footprint just because it is allowed. A square
15 cm chassis has a diagonal of about 21.2 cm. For a centre pivot it sweeps a
larger width than 15 cm; an off-centre drive axle can need even more clearance.
Physically check the complete turning envelope, including wheels and sensors,
at every 90/180-degree corner before powering a floor test.

## 3. Wire only after checking module voltage requirements

The following GPIOs match Config.h and avoid the WROVER-IE's PSRAM pins
GPIO16/17 and flash pins GPIO6–11. Confirm they are available on your board's
headers. These are GPIO numbers, not physical header positions; this map is
not for ESP32-C3/S3 or a board with other peripherals already using these pins.

| Module signal | ESP32 GPIO |
| --- | --- |
| All ToF SDA + chosen gyro SDA | 21 |
| All ToF SCL + chosen gyro SCL | 22 |
| Left ToF XSHUT | 13 |
| Front ToF XSHUT | 14 |
| Right ToF XSHUT | 23 |
| L298N ENA, left motor PWM | 25 |
| L298N IN1 / IN2, left direction | 26 / 27 |
| L298N ENB, right motor PWM | 33 |
| L298N IN3 / IN4, right direction | 18 / 19 |
| KY-033 digital OUT/S | 32 |
| Optional start button, other side to GND | 4 |

Use common ground between controller, sensors, driver and power supply. Keep
motor-current return wiring away from thin sensor wiring where practical.

### Power safeguards

- ESP32 GPIO is 3.3 V logic. Never put a 5 V sensor output on it directly.
- Do not connect the 9 V battery directly to ESP32's 3V3 or 5V pin.
- Verify a regulated supply for your exact ESP32 development board and sensors.
  Do not power the motors from an ESP32 GPIO, 3V3 rail or USB power rail.
- A compatible ToF breakout can be supplied at 3.3 V; verify the actual breakout
  supports this. Raw chips and differently designed breakouts have different needs.
- Check the I2C pull-ups go to 3.3 V-compatible logic, not an unknown 5 V rail.
- Power the KY-033 at 3.3 V, connect GND and OUT/S to GPIO32, and keep digital
  mode selected. Its internal GPIO pull-up is enabled. Do not connect an
  externally driven 5 V output directly to the ESP32.
- For the MPU-9250/6500 breakout, verify its 3.3 V supply and I2C wiring. Use
  the level-shifted VIN/SDA/SCL interface of the pictured Adafruit ICM-20948
  breakout if you choose that alternative. A raw ICM-20948 has 1.8 V I/O and
  is not interchangeable with that breakout's safe 3.3 V interface.
- The code drives XSHUT LOW, then releases it as an input. This relies on a
  breakout pull-up; add the appropriate pull-up only after checking its schematic.
- Remove the L298N **ENA and ENB jumpers** before attaching PWM. Its separate
  regulator/5V-enable jumper is a different item; check that module's power rules.
- Add hardware pull-downs (for example 10 kOhm, if compatible with your module)
  to ENA/ENB so motors remain disabled while the ESP32 resets. Use a reachable
  motor-power switch. Software alone cannot guarantee reset-time behaviour.
- Verify the L298N module includes the required flyback protection. Match motor
  supply to motor ratings and account for the driver's voltage drop and heat.
- Do not assume the listed battery can supply two motors at startup/stall.
  Measure supply voltage under load; resets and ToF faults can be power problems.
- Avoid connecting competing USB and external power sources unless the board's
  documented power arrangement supports it. Do not back-feed a USB supply.

The power circuit is not confirmed by the inventory. Identify the TT motor
voltage rating, battery chemistry/holder wiring and startup current before
connecting it. A rectangular 9 V battery is not a proven two-motor supply.
If the 3.7 V cells are lithium-ion, a series pair reaches 8.4 V when fully
charged: use suitable protection/charging and regulation for the actual motor
and board ratings. PWM and the L298N's variable voltage drop are not voltage
regulators. Do not use the L298N 5 V regulator jumper as an assumed ESP32 power
solution, particularly at a low motor-supply voltage. Obtain/verify a suitable
regulated controller supply; it is not established by this parts list.

### Three ToFs on one I2C bus

VL53L0Xs and VL53L1Xs start at address 0x29. The program holds all in shutdown, enables
one at a time and assigns **left 0x30, front 0x31, right 0x32** at every boot.
The gyro probes 0x68 and 0x69 automatically, so either AD0 setting is accepted.
The three ToFs can be either supported family, including a mixed set. L1Xs use
Short mode, a 20 ms timing budget and 50 ms continuous period. L0Xs also range
every 50 ms. Initialization prints the detected families and assigned addresses.
If your sensors lack XSHUT or are an unsupported model, adapt the hardware layer
or use an appropriate I2C multiplexer. Do not just connect three identical
default-address devices and expect them to work independently.

## 4. First upload: sensor-only checks

Keep `AllowMotorMotion = false` and `NavigationCalibrated = false` in Config.h.
Disconnect motor power while uploading. Hold the stationary chassis still
during startup gyro calibration: about 1.2 seconds of fresh samples, plus sensor
initialization. For VS Code, follow [VSCODE_SETUP.md](VSCODE_SETUP.md): PlatformIO
installs the two ToF libraries automatically. For Arduino IDE, install them
through Library Manager as described in README.md.

At 115200 baud the telemetry resembles:

```text
IDLE L=70(0) F=500(0) R=45(0) yaw=0.0 rate=0.0 IR=1 white=0 ok=1
```

This is an illustrative output, not a measurement from your robot. ToF brackets
show range-status values: 0 is accepted as valid; other statuses stop navigation.
`ok=1` requires all three recent valid ranges and a healthy gyro. `white` is the
configured IR classification. `yaw` is integrated degrees, positive to the LEFT.

1. Put a wall at known distances in front of each sensor. Check correct identity,
   plausible millimetres and no chassis reflections. Repeat near a wall edge.
2. Place the robot on black and then on white tape. Record the raw IR readings.
   Set `WhiteDigitalLevel`, or select analog mode and a measured threshold.
   For the KY-033, keep digital mode; its trim potentiometer and mounting height
   determine the threshold. Recheck in the actual room lighting and with motors
   later running. A different analog module requires compatible voltage and
   verified analog-capable GPIO wiring before selecting analog mode.
3. Turn the chassis by hand left 90 degrees: yaw should rise by about +90.
   Right 90 degrees should change it by about -90. Correct `GyroSign` if needed.
4. Keep it stationary and inspect drift. Send `C` to recalibrate while stopped.
   Calibration detects changing rate, but cannot detect every steady movement;
   physically keeping the robot still is essential.
5. Fault-injection tests with motor power OFF: disconnect a ToF or gyro and verify
   `ok=0`. Do not interpret a failed reading as a route being available.

**Do not enable motor motion if these tests fail.**

## 5. Motor checks: wheels lifted

After voltage/wiring checks pass, set `AllowMotorMotion = true`, leaving
`NavigationCalibrated = false`. Reupload with motor power disconnected, then
support the chassis with both drive wheels clear of the surface before power-on.

| Command | Result |
| --- | --- |
| H | Print help |
| Q | Stop motor commands and return to idle |
| C | Stop and recalibrate gyro while stationary |
| F | Both wheels forward for 300 ms; BENCH ONLY |
| B | Both wheels backwards for 300 ms; BENCH ONLY |
| T | Gyro-controlled 90-degree right turn, then idle |
| Y | Gyro-controlled 90-degree left turn, then idle |
| U | Gyro-controlled 180-degree left turn, then idle |
| S | Full navigation; refused until both safety flags are true |

Use F/B only with wheels lifted. Check each positive wheel command pushes the
robot forward; adjust `LeftMotorSign` / `RightMotorSign` rather than guessing
motor colour polarity. Check minimum PWM that moves each loaded motor reliably.

Do not run gyro turn tests with wheels lifted: the chassis would not rotate and
the test would time out. After direction checks pass, use T/Y/U on the floor in
a clear, sufficiently large area with valid sensor returns and the power switch
within reach. Start slowly and mark floor angles with tape. A gyro and PWM alone
do not remove motor inertia or wheel slip; tune gains, minimum turn PWM and
settling thresholds based on repeated real tests.

Zero PWM currently disables the bridges and COASTS, rather than actively braking.
Measure actual stopping distance; stopping commands do not stop inertia instantly.
Serial Q is only useful while connected. The hardware power switch is the
independent stop mechanism during untethered runs.

## 6. Calibrate straight driving and junction geometry

First construct a test corridor of the official width. Adjust the following
against the complete chassis dimensions and measured readings:

| Setting | How to choose it |
| --- | --- |
| TargetWallMm | Desired spacing to the selected wall, including sensor inset and safe chassis clearance |
| SideOpeningMm | Between closed-side readings and readings at real side openings |
| WallPresentMm | Range within which feedback from the selected wall is useful; below opening threshold |
| FrontDecisionMm | Far enough from a front wall to stop and turn with chassis clearance |
| FrontEmergencyMm | Above sensor-to-front-edge offset plus stopping distance and safety margin |
| SideEmergencyMm / TurnEmergencyMm | Based on sensor offsets, body clearance and swept corner geometry |
| MinimumRangeMm / MaximumRangeMm | 30–1200 mm by default from inventory; extend only after validating the exact sensors |
| CruisePwm / SlowPwm | Lowest useful loaded speed first; speed is not measured without encoders |
| WallGain / HeadingGain | Small corrections that do not oscillate or scrape walls |
| JunctionAdvanceMs | Measured time to put the drive axle at the preferred-side turning position |
| JunctionExitMs | Measured travel time to clear the same opening after completing a turn |

The example values are NOT tuned to your unmeasured corridor or chassis.
Side/turn emergency defaults are 35 mm, above the listed 30 mm minimum reliable
range. Target right spacing is 45 mm as a starting example. If sensor inset or
stopping distance needs more clearance, increase both and recheck openings.

When the side sensor first sees a gap, the drive axle may still be near the
entrance edge of that perpendicular passage. Rotating immediately can hit the
corner. Forward sensor offset and the three-frame opening confirmation also
shift where the detection occurs. Use tape marks to measure the axle position
when an opening is CONFIRMED, then determine the needed approach distance.

For an initial estimate use `time = distance / measured speed`, but calibrate
the result at the exact slow PWM, battery load and floor surface being used.
The approach ends early if the front reaches `FrontDecisionMm`; if that point
is not geometrically safe for a pivot, change the chassis/settings first.
Blindly changing a delay is not a substitute for measuring the geometry.

This code ignores new junction decisions during a calibrated exit interval,
preventing an immediate second turn into the opening just entered. If this
interval is too short the bot can re-turn; too long and it can miss the next
opening. This is a known limitation without distance feedback.

After checking these items, set `NavigationCalibrated = true`. Send C while
stationary, wait for `ok=1`, then S. The bot has a three-second start delay.
An optional button on GPIO4 starts from idle; another press stops the run.
On a fault or finish, Q returns to idle. Never restart an official attempt
unless the organiser permits it. Power-cycle after fixing an initialization failure.

## 7. How the algorithm works

The table below is for the default RIGHT strategy. Set FollowLeftWall=true in
Config.h to mirror the strategy to LEFT: left > straight > right > back. Wall
spacing also follows the selected side; do not change physical sensor labels or
GyroSign. See CONTINUE_YOUR_SKETCH.md for both decision tables and wiring fixes.

| Observation in normal following | Action |
| --- | --- |
| Confirmed right opening | Align for junction, turn right 90 degrees, clear junction |
| Right closed, forward clear | Drive forward and correct right-wall spacing |
| Right closed, forward blocked, left open | Turn left 90 degrees, clear junction |
| All three blocked | Turn 180 degrees and leave the dead end |
| Confirmed white finish after black floor | Stop and stay finished |
| Invalid/stale ranges, gyro fault, timeout or clearance limit | Stop and latch a fault |

Opening confirmations count NEW complete ToF frames, not loop iterations.
Motor control and turn/approach/exit states do not use long blocking delays.
Stationary initialization and gyro calibration do contain short delays with
motor commands disabled. During turns, a target yaw is measured relative to
the start of that turn; fixed motor-on time does not decide the angle.

Wall correction uses the right range. Too close: right wheel gets faster and
left slower, steering away. Too far: the opposite. If the right wall disappears,
gyro heading keeps a short straight segment while an opening is confirmed.

Finish detection is checked during moving states, not just at intersections.
It first requires 200 ms of black, then 40 ms continuous white; both are
configurable. A narrow finish stripe must remain under the IR detector for
longer than this confirmation time. Check stripe width / actual travel speed,
IR location, stopping distance and the organiser's definition of crossing.

## 8. What this does NOT guarantee

- No shortest-path guarantee and no arbitrary-maze completion guarantee.
- No route memory or loop recognition. The run timeout stops a suspected loop;
  it does not solve it. A grid/map with visited-junction tracking is a later
  upgrade, ideally with wheel encoders and confirmed cell dimensions.
- No accurate forward position from the gyro. Double-integrating its
  acceleration is not implemented as a shortcut for encoders.
- No collision-proof guarantee from three narrow distance beams. Chassis
  corners can hit walls outside those beams; check turning envelope physically.
- Out-of-range/no-return results stop the bot. Do not globally turn all error
  statuses into 'open'; distinguish valid no-target results for your exact
  sensor and scene before changing this policy.
- No battery-voltage monitor, encoder stall detection or independent firmware
  watchdog has been implemented. Use adequate power, reliable connections,
  conservative speeds and a physical power switch.

If the organiser allows looped mazes with an internal finish, use this to develop
the sensor/motor layer, but plan a map/exploration upgrade. That can happen
during the single official attempt; it does not require a preliminary run.

## 9. Two-person work split and week-5 checkpoints

| Person | Ownership | Handover requirement |
| --- | --- | --- |
| You | Chassis, mounting, power, sensor and motor tests; Sensors.h / Motors.h | Verified wiring table, readings, motor/gyro signs, measured clearances |
| Friend | Navigation.h, finish behaviour, host tests and test-case design | Passing tests, clear parameter changes and reproducible runs |
| Both | Config.h review and physical integration | Agree one configuration; record battery condition and commit SHA |

Swap these roles if desired; avoid both editing Config.h without coordinating.

| Checkpoint | Minimum result before moving on |
| --- | --- |
| First session | Confirm models, course width, topology and power plan; agree branches |
| Week 1 | Stable ToF/gyro/IR telemetry and verified motor direction |
| Week 2 | Straight corridor and repeatable 90/180-degree turns without contact |
| Week 3 | Right/left corners, T-junctions and dead ends on your own course |
| Week 4 | Unseen arrangements of your test walls plus finish and fault tests |
| Week 5 | Freeze proven settings; repeat reliability tests and check size/rules |

The inventory requires 3D-print requests at least **three days before the
week-5 technical meeting**. Submit the measured chassis and sensor mounts in
time; use the actual meeting date to set that deadline.

If you are already later than week 1, use these as ordered milestones rather
than pretending the earlier weeks are still available. Do not rush to higher
speed before full-course success is repeatable with different battery states.

## 10. Add to your GitHub project without overwriting your work

Inside your existing local clone, first inspect the branch and changes:

```sh
git status
git switch -c feature/right-wall-starter
```

Copy only the supplied `firmware/`, `docs/` and `tests/` files into the repository.
Compare any existing files with the same names before replacing them. Append
relevant README information yourself, retaining your authorship/project history.
Leave Main.c alone until you and your friend review what it contains. The Arduino
entry point is the .ino file here, not C++ classes pasted into a .c file.

```sh
git add firmware/Micromouse docs tests
git commit -m "Add right-wall-following starter and calibration guide"
git push -u origin feature/right-wall-starter
```

Review together before merging into main. Each subsequent feature uses its own
branch, such as `feature/sensor-checks`, `feature/motor-calibration` or
`feature/junction-tuning`. These are instructions for you; no push has been made here.

Record each real test using TEST_LOG.md. Include the commit SHA, configuration,
battery condition, layout, result and one next change. Change one tuning item
at a time where possible.

## Technical references used

- [Espressif LEDC / Arduino-ESP32 PWM APIs](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [Espressif GPIO voltage guidance](https://docs.espressif.com/projects/esp-faq/en/latest/hardware-related/hardware-design.html)
- [Adafruit VL53L0X library API](https://github.com/adafruit/Adafruit_VL53L0X/blob/master/src/Adafruit_VL53L0X.h)
- [Pololu VL53L0X carrier connections and voltage details](https://www.pololu.com/product/2490)
- [Espressif WROVER-E/IE datasheet and PSRAM pins](https://documentation.espressif.com/esp32-wrover-e_esp32-wrover-ie_datasheet_en.html)
- [Pololu VL53L1X Arduino library](https://github.com/pololu/vl53l1x-arduino)
- [InvenSense MPU-9250 register map](https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf)
- [InvenSense MPU-6500 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)
- [InvenSense ICM-20948 datasheet / register banks](https://cdn.sparkfun.com/assets/learn_tutorials/8/9/3/DS-000189-ICM-20948-v1.3.pdf)
- [Joy-IT KY-033 reflectance module](https://sensorkit.joy-it.net/en/sensors/ky-033)
- [ST L298 datasheet](https://www.st.com/resource/en/datasheet/l298.pdf)
- [Micromelon wall-following and gyro-turn activity](https://www.micromelon.com.au/resources/maze-iii)

Vendor breakout specifications are examples only: check the exact modules you own.
