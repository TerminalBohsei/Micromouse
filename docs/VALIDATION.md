# Validation record

## Checks completed in the development environment

- `tests/navigation_test.cpp` compiled as C++11 with `-Wall -Wextra -Werror -pedantic`.
- All 82 navigation assertions passed after the inventory-driven clearance changes.
- All 33 additional physical LEFT-wall assertions passed: selected-side priority,
  steering signs, nonzero starting yaw, turn overshoot/settling, the opposite-side
  corner, a U-turn, physical serial-test directions, finish and clearance stop.
- `tests/gyro_test.cpp` compiled with the same C++11 warning flags; all 46
  gyro/configuration assertions passed. Fixtures cover MPU-6500, MPU-9250,
  ICM-20948 at both addresses, positive/negative rate decoding, missing data,
  short/failed transfers, configuration failure/readback mismatch, clipped
  readings, unknown chips and duplicate/WROVER-reserved pin assignments.
- The navigation suites passed with AddressSanitizer and UndefinedBehaviorSanitizer
  after adding the wall-side setting; the gyro suite also passed those checks
  in the previous inventory update.
  Leak detection was disabled because the execution environment blocks the
  process inspection it needs; the navigation controller performs no dynamic allocation.
- The complete .ino and headers passed C++11 syntax checks for both the 2.x
  and 3.x PWM code paths against compile-only hardware interface doubles.
  Those doubles were not packaged as production code or substituted for real libraries.
- The smaller docs/FirstUpload.ino.txt sensor-only sketch passed the same interface
  syntax check with all required headers present.
- Public vendor documentation was checked for the distance/PWM APIs and gyro
  register maps. The inventory's module photos and labels informed the pin map
  and supported sensor choices; they do not prove what is installed on the bot.

## Not completed here

- Actual compilation/linking against an installed ESP32 Arduino core and real
  Adafruit/Pololu libraries. Arduino CLI/PlatformIO and those dependencies were not installed.
- Flashing the team's ESP32, verifying exact component variants, or motor power tests.
- Real ToF optical tests, floor IR calibration, gyro drift/turn accuracy tests.
- Chassis clearance, junction approach/exit calibration, speed or maze-completion tests.
- Reading the current live GitHub repository or changing/pushing existing files.

Desktop logic tests cannot prove real-world maze completion or collision avoidance.
The next required check is an Arduino IDE build for the exact board, followed by
the ordered hardware checklist in START_HERE.md with motor power initially off.

The revised package changes XSHUT to GPIO13/14/23, KY-033 OUT to GPIO32, and
the optional start button to GPIO4. Rewire to this map before uploading it;
the previous GPIO16/17 map is unsuitable for the inventory's WROVER-IE.

The continuation preserves setup(), printReading() and loop(), and adds
FollowLeftWall=false (RIGHT default) / true (LEFT). Shared calibration names are
now TargetWallMm, WallPresentMm and JunctionAdvanceMs. These names apply to the
selected side, while L/F/R telemetry and positive-left gyro signs stay physical.
