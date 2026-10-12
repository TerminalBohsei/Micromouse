# Wiring

Pin numbers match `Config.h`. If you wire differently, change them there.
**Check this table against your build before powering up.**

## I2C bus (all three ToF sensors + gyro share it)

| Signal | ESP32 pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Power every I2C device from 3.3 V and common GND.

| Device | Address | Extra pin |
|---|---|---|
| VL53L0X left  | 0x30 (set at boot) | XSHUT -> GPIO 25 |
| VL53L0X front | 0x31 (set at boot) | XSHUT -> GPIO 26 |
| VL53L0X right | 0x32 (set at boot) | XSHUT -> GPIO 27 |
| MPU-6050 | 0x68 | AD0 -> GND |

All three ToF sensors power up at 0x29. The code holds them in reset via
XSHUT and wakes them one at a time to assign the addresses above.

## L298N

| L298N | ESP32 | Notes |
|---|---|---|
| ENA | GPIO 14 | left motor PWM (remove the jumper cap) |
| IN1 | GPIO 32 | left direction |
| IN2 | GPIO 33 | left direction |
| ENB | GPIO 13 | right motor PWM (remove the jumper cap) |
| IN3 | GPIO 18 | right direction |
| IN4 | GPIO 19 | right direction |
| GND | GND | **must be shared with the ESP32** |

## Other

| Part | ESP32 | Notes |
|---|---|---|
| IR floor sensor (analog out) | GPIO 34 | input-only ADC1 pin |
| Start button | GPIO 0 | the BOOT button (to GND when pressed) |
| LED | GPIO 2 | on-board LED |

## Mounting notes

- ToF sensors: left and right facing straight out sideways, front straight
  ahead, all at the same height, with nothing blocking the beam.
- MPU-6050: mount flat, Z axis up, away from the motors. If turning left gives
  a negative heading, set `GYRO_SIGN = -1`.
- IR sensor: point at the floor 3-10 mm above it, ahead of the wheels.
- Keep the whole robot within 15 x 15 cm.
