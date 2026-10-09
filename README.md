# Micromouse: wall-following starter

For **Visual Studio Code**, start with [docs/VSCODE_SETUP.md](docs/VSCODE_SETUP.md).
For Kohsei's draft sketch, start with [docs/CONTINUE_YOUR_SKETCH.md](docs/CONTINUE_YOUR_SKETCH.md),
then follow [docs/START_HERE.md](docs/START_HERE.md). This is a first implementation
updated against the team's nine-page parts inventory, not a robot that has been physically tested.

## Selected parts from the inventory

| Function | Use | Firmware support |
| --- | --- | --- |
| Controller | ESP32 development board; inventory photo shows WROVER-IE | Classic ESP32, pins avoid GPIO16/17 reserved for PSRAM |
| Drive | Two TT geared DC motors and wheels, plus a suitable passive support | Differential steering through the L298N |
| Motor driver | L298N module | Independent ENA/ENB PWM; remove those two jumpers |
| Wall distances | Three breakouts labelled VL53L0/1XV2, with XSHUT | Detects each VL53L0X or VL53L1X at boot |
| Turns | ONE MPU-9250/6500 module, or the Adafruit ICM-20948 breakout | Detects MPU-6500/9250 or ICM-20948 at 0x68/0x69 |
| Finish line | KY-033 line tracking module, facing DOWN | Digital reflectance; measure black/white polarity |
| Power | Inventory has 9 V and two 3.7 V cells | Supply design remains to be verified; see START_HERE |

The inventory lists available options, not a confirmed assembled robot. Check
the markings on your actual board and startup model report. Unknown gyro/ToF
identities refuse initialization. No MPU6050 library is required: checked gyro
register transfers are implemented in GyroDriver.h and RegisterIO.h.
The IR-08H obstacle detector, ultrasonic sensors, stepper and ULN2003 are not
used by this version. A rotary encoder in the assortment is not automatically
an installed wheel encoder. Analog floor mode is retained only for a different
verified reflectance module; the selected KY-033 uses its digital output.

## Rules used

- Robot smaller than 15 cm x 15 cm x 25 cm; all attached parts count.
- Maze footprint 2 m x 2 m; wall height 25 cm.
- Black floor, white walls and white finish marking.
- No known map or rehearsal of the official maze; one official attempt.
- No detachable components, flight, or climbing over walls.
- Wall contact attracts a time penalty.
- Complete by week 5; use the team's actual competition date for scheduling.

These override the pasted README's older left-hand strategy and 100 cm height.
The live repository could not be retrieved in this environment. Nothing has
been pushed to GitHub and existing Main.c/README.md contents were not changed.

## Code

| File | Responsibility |
| --- | --- |
| platformio.ini | Pinned ESP32 PlatformIO build, sensor dependencies and serial settings |
| src/main.cpp | VS Code/PlatformIO entry point compiling the same firmware sketch |
| firmware/Micromouse/Micromouse.ino | Startup, serial commands, telemetry and optional start button |
| firmware/Micromouse/Config.h | Pins, safety gates and calibration constants |
| firmware/Micromouse/Sensors.h | Three ToFs, gyro calibration/yaw and floor reflectance |
| firmware/Micromouse/GyroDriver.h / RegisterIO.h | Model detection, configuration and complete I2C transfers |
| firmware/Micromouse/Motors.h | Signed left/right PWM through the L298N |
| firmware/Micromouse/Navigation.h | Selectable wall-hand rule and non-blocking motion states |
| tests/navigation_test.cpp | Hardware-independent decision/state tests |
| tests/gyro_test.cpp | Gyro identity/rate/failure fixtures and pin-map checks |
| tests/left_navigation_test.cpp | Physical left-wall decisions, steering and turn checks |
| docs/FirstUpload.ino.txt | Optional smaller sensor-only sketch; use with the supplied headers |
| docs/TEST_LOG.md | Reproducible physical-test record |

## Visual Studio Code / PlatformIO

Open this folder (the one containing `platformio.ini`) in local VS Code and
install the official PlatformIO IDE extension. Follow
[docs/VSCODE_SETUP.md](docs/VSCODE_SETUP.md) for Windows build, COM port upload
and sensor tests. The first build installs the ESP32 tools and both libraries.
The supplied generic `esp32dev` profile assumes classic ESP32 with 4 MB flash;
verify the exact development board. Keep both motion/calibration gates false
for the first upload and disconnect motor power.

## Arduino IDE (alternative)

1. Install/select the Espressif ESP32 board package and the exact board you own.
2. Install **Adafruit VL53L0X** and **VL53L1X by Pololu** through Library Manager,
   including all dependencies it offers. Both are required for automatic ToF detection.
   This code does not use the Adafruit VL53L1X library or any external gyro library.
3. Open `firmware/Micromouse/Micromouse.ino` and keep the other headers in that folder.
4. Select the correct board/USB port and upload with motor power disconnected.
5. Open Serial Monitor at **115200 baud**. Line ending can be Newline; CR/LF are ignored.
6. Verify readings with motor motion still disabled. Follow START_HERE before enabling it.

Motor PWM supports Arduino-ESP32 2.x and 3.x using their respective LEDC APIs.
This is not a claim that an actual ESP32 build has already passed here.

## Desktop tests / Codespaces

From this starter directory:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/navigation_test.cpp -o /tmp/micromouse_test
/tmp/micromouse_test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/gyro_test.cpp -o /tmp/micromouse_gyro_test
/tmp/micromouse_gyro_test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/left_navigation_test.cpp -o /tmp/micromouse_left_test
/tmp/micromouse_left_test
```

Codespaces is useful for editing and these tests. For USB upload, use local
VS Code with PlatformIO or Arduino IDE on the computer connected to the ESP32.

## Important limitations

Set FollowLeftWall=false for RIGHT (default), or true for LEFT in Config.h.
TargetWallMm and JunctionAdvanceMs apply to the selected side. Physical
sensor labels and gyro sign do not change when selecting a wall.

This is a reactive wall follower, not a mapped-maze solver. Wall following
does not guarantee an arbitrary internal finish in a maze containing wall islands
or loops. Confirm the official maze topology before relying on this strategy.

No installed wheel encoders are confirmed. The gyro estimates rotation, not forward distance
or reliable x/y position. Junction advances and exits therefore use calibrated
travel times and will change with battery voltage, floor grip and motor loading.
Real maze/chassis tests, not just desktop tests, are necessary before competition.

Invalid/out-of-range ToF results conservatively stop navigation; they are not
treated as openings. The initial version may stop in an open area where a sensor
cannot produce a valid return. Diagnose this on your test course before choosing
any model-specific handling for a confirmed no-target result.
Accepted distances default to the inventory's conservative **30–1200 mm** envelope.
Do not expect a valid 2 m reading just because the overall maze is 2 m wide.
