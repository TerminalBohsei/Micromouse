# Program the bot from Visual Studio Code on Windows

Use **Visual Studio Code** (the blue icon) with the official **PlatformIO IDE**
extension. PlatformIO provides the ESP32 compiler, Arduino framework, library
manager, uploader and serial monitor inside VS Code. Arduino IDE is optional.

Run these steps in local VS Code on the computer connected to the robot's USB
cable. GitHub and Codespaces can hold/edit the project; the commands below use
the local computer's COM port.

## 1. Install the extension

1. Install [Microsoft Visual Studio Code](https://code.visualstudio.com/).
2. Open it, press **Ctrl+Shift+X**, search **PlatformIO IDE**, and install the
   official extension published by **PlatformIO**.
3. Let setup finish and reload VS Code if prompted. Its PlatformIO Core is
   included; you do not need to install a separate global `pio` command.

## 2. Open the project folder

Extract the starter ZIP. Choose **File > Open Folder**, then select
`Micromouse_Starter`: the folder that directly contains `platformio.ini`,
`src`, `firmware`, `docs` and `tests`. If you add this starter as a subfolder
of your Git repository, open that subfolder in VS Code for these steps.

This is already a PlatformIO project; you can use it immediately without
creating a second project through New Project.

| File | What you edit |
| --- | --- |
| `platformio.ini` | Board, dependency versions, serial settings and optional local port |
| `firmware/Micromouse/Config.h` | Wiring, motor safety gates and calibration |
| `firmware/Micromouse/Micromouse.ino` | Arduino setup/loop and serial commands |
| Other headers in `firmware/Micromouse` | Sensor, motor and navigation implementation |
| `src/main.cpp` | Existing entry point includes the firmware; usually leave it as supplied |

`src/main.cpp` compiles the same sketch used by Arduino IDE. Keep one firmware
copy. A `Main.c` outside `src` is not part of this PlatformIO build. If you add
another file defining `setup()` or `loop()` inside `src`, remove that duplicate.

## 3. Confirm the board and keep motion disabled

The supplied `board = esp32dev` profile assumes a **classic ESP32 with 4 MB
flash**. The inventory photo shows a WROVER-IE module, but not enough information
to identify the complete development board. This program does not require
PSRAM and avoids WROVER GPIO16/17. Verify your board markings and flash size.
For a different board, choose its matching PlatformIO board ID; an ESP32-C3/S3
also requires reviewing this project's GPIO map.

In `firmware/Micromouse/Config.h`, leave these existing settings unchanged:

```cpp
constexpr bool AllowMotorMotion = false;
constexpr bool NavigationCalibrated = false;
```

Check the wiring against [START_HERE.md](START_HERE.md). Disconnect motor power
for the initial build/upload and sensor tests.

## 4. Build inside VS Code

Click the PlatformIO **ant icon** on the left. Under **Quick Access >
Miscellaneous**, open **PlatformIO Core CLI**. The terminal's current directory
must be the folder containing `platformio.ini`; use `cd` to that folder if needed.

Run:

```powershell
pio run -e micromouse
```

The first build downloads the pinned ESP32 platform/toolchain and both ToF
libraries, plus declared dependencies. Internet access is needed for those
downloads. Wait for a green **SUCCESS** result before uploading. Later builds
reuse the downloaded packages. You can also use PlatformIO Project Tasks >
micromouse > General > Build.

The platform is pinned to 6.12.0, which uses Arduino-ESP32 2.0.17. The firmware
also has the 3.x PWM branch, but this configuration builds the 2.x branch.

## 5. Find the connected ESP32 port

Connect the ESP32 with a **USB data cable**, keeping motor power disconnected.
Run in the same PlatformIO terminal:

```powershell
pio device list
```

Find the port that appears when you connect the board. If needed, compare the
list before and after connecting it, or check Windows Device Manager > Ports
(COM & LPT). The examples below use **COM4**; replace it with your actual port.
Ports can change after reconnecting. Close Arduino IDE serial monitors/plotters
and other applications using that port.

## 6. Upload the firmware

Keep the chassis stationary for startup gyro calibration. Run:

```powershell
pio run -e micromouse -t upload --upload-port COM4
```

Wait for **SUCCESS**. Uploading writes the compiled program into the ESP32's
flash; it remains there after USB is unplugged. The program starts again when
the board resets or receives power. Copying files to GitHub does not flash it.

## 7. Read sensors and send commands

Open the serial monitor:

```powershell
pio device monitor -e micromouse --port COM4 --baud 115200
```

The project buffers your command until **Enter**. Type `H`, then Enter, for
help. The continuous telemetry should resemble:

```text
IDLE L=70(0) F=500(0) R=45(0) yaw=0.0 rate=0.0 IR=1 white=0 ok=1
```

This is example output, not a measurement from your robot. Distances are
millimetres. Move a wall in front of each ToF and check that the correct L/F/R
value changes. Check black floor versus white tape. Rotate the chassis by hand
and check yaw direction. Send `C` then Enter to recalibrate while stationary.
`ok=1` needs fresh valid ToF readings and a healthy gyro; investigate `ok=0`
before enabling motors. If startup messages were missed, `H` still prints help
and the live readings continue. Initialization errors require fixing wiring
and restarting the board.

Keep 115200 on the monitor because the sketch uses `Serial.begin(115200)`.
This monitor baud rate is separate from the USB upload speed.

Press **Ctrl+C** to close the monitor before the next upload. When testing with
motor motion enabled later, send `Q` then Enter **before** closing it: Ctrl+C
only closes the terminal monitor and does not command the robot to stop.
Keep the hardware motor-power switch within reach during powered motion tests.

## 8. Change code, rebuild and test

Edit the firmware or `Config.h`, save, close the serial monitor, upload again,
then reopen the monitor. Follow [START_HERE.md](START_HERE.md) in order for lifted
wheel pulses, floor turn tests and corridor calibration. Only enable full
navigation once that checklist passes. Configuration values are compiled into
the firmware, so saving a file alone does not update the robot.

Commit `platformio.ini`, `src`, `firmware`, `docs` and `tests` to your Git
repository. The supplied `.gitignore` excludes generated build downloads and
local editor state. Each teammate installs PlatformIO locally and uses their
own USB port. Explicit `--upload-port`/`--port` arguments keep a local COM choice
out of your shared configuration.

## Troubleshooting

| Message or symptom | Next action |
| --- | --- |
| `pio` is not recognised | Open **PlatformIO Core CLI** using the ant icon; an ordinary terminal may not have `pio` on PATH |
| No `platformio.ini` found | Change the terminal directory to `Micromouse_Starter`, or open that folder in VS Code |
| COM port access denied | Close all monitors/plotters or other programs using it, reconnect the board, list ports again and retry |
| No new port appears | Try a known USB data cable and another USB port; identify the board's USB-to-serial chip before installing its appropriate driver |
| Repeated upload connection failures | Recheck actual board ID and USB port, motor power disconnected, then follow the exact board's documented manual boot/upload procedure |
| Garbled monitor text | Use 115200 baud for this firmware |
| ToF/gyro init failure or `ok=0` | Check supply, common ground, SDA/SCL, separate XSHUT wires and supported sensor models; restart after fixing initialization failures |
| Start command refused | Default safety gates are false; complete the physical checklist before changing them |

## Official references

- [PlatformIO in Visual Studio Code](https://docs.platformio.org/en/latest/integration/ide/vscode.html)
- [Generic classic ESP32 board profile](https://docs.platformio.org/en/latest/boards/espressif32/esp32dev.html)
- [Library dependency configuration](https://docs.platformio.org/en/latest/projectconf/sections/env/options/library/lib_deps.html)
- [Serial device list](https://docs.platformio.org/en/latest/core/userguide/device/cmd_list.html)
- [Serial monitor](https://docs.platformio.org/en/latest/core/userguide/device/cmd_monitor.html)
- [Espressif32 platform releases](https://github.com/platformio/platform-espressif32/releases)
