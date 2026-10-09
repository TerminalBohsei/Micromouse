# Physical test log

Copy this section for each test. Record observations, not just whether it worked.

## Test ID / date / tester

- Git commit SHA:
- ESP32 / ToF / gyro / IR / driver models:
- Battery type and loaded voltage:
- Corridor width / wall height / finish-strip width:
- Robot width / length / drive axle and sensor offsets:
- Firmware version and changed Config.h settings:
- Scenario: straight / right corner / left corner / T / cross / dead end / finish / fault:
- Repetitions and success count:
- Wall contacts:
- Measured turn angles and stopping distance:
- Sensor statuses / unexpected telemetry:
- Observation or video reference:
- One change to test next:

## Required test cases

- [ ] Each ToF matches its physical direction and returns plausible distances.
- [ ] Startup reports the expected gyro and all three ToF families/addresses.
- [ ] WROVER GPIO16/17 are unused; all wiring matches the revised Config.h.
- [ ] Black/white IR classification is stable at the final mounting height.
- [ ] Positive motor commands drive both wheels forward.
- [ ] Gyro yaw rises on left rotation and falls on right rotation.
- [ ] Supply does not reset controller when motors start or reverse.
- [ ] Right wall following starts at several different offsets without scraping.
- [ ] Straight motion remains stable through brief missing right-wall contact.
- [ ] Right opening beats straight/left when several routes are available.
- [ ] Left corner works when right and front are blocked.
- [ ] Dead end produces a 180-degree turn without touching a wall.
- [ ] Adjacent junctions are not missed by the exit lockout.
- [ ] Finish strip stops the bot at minimum/maximum intended speed.
- [ ] Short white noise/reflections do not trigger a finish.
- [ ] Disconnected/stale sensor causes stopped commands, not a turn into an opening.
- [ ] Turn timeout and physical power-off are independently tested safely.
- [ ] Several unfamiliar test layouts succeed repeatedly with realistic battery states.
- [ ] Complete robot fits below 15 cm x 15 cm x 25 cm and its swept path clears corners.
