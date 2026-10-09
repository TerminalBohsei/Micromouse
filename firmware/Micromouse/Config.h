// Micromouse configuration. All dimensions and speeds below need physical calibration.
#pragma once
#include <stdint.h>

namespace Config
{
    // Keep both false until the checks in docs/START_HERE.md are complete.
    constexpr bool AllowMotorMotion = false;
    constexpr bool NavigationCalibrated = false;
    constexpr bool FollowLeftWall = false; // false = RIGHT wall; true = LEFT wall.

    // Inventory photo: classic ESP32-WROVER-IE. GPIO16/17 belong to PSRAM.
    // Confirm these numbered GPIOs exist on your actual development board.
    constexpr int SdaPin = 21;
    constexpr int SclPin = 22;
    constexpr int LeftXshutPin = 13;
    constexpr int FrontXshutPin = 14;
    constexpr int RightXshutPin = 23;
    constexpr int LeftEnablePin = 25; // L298N ENA: motor 1 speed.
    constexpr int LeftIn1Pin = 26; // motor1pin1 -> L298N IN1.
    constexpr int LeftIn2Pin = 27; // motor1pin2 -> L298N IN2.
    constexpr int RightEnablePin = 33; // L298N ENB: motor 2 speed.
    constexpr int RightIn1Pin = 18; // motor2pin1 -> L298N IN3.
    constexpr int RightIn2Pin = 19; // motor2pin2 -> L298N IN4.
    constexpr int FloorPin = 32; // KY-033 digital OUT, powered at 3.3 V.
    constexpr int StartPin = 4; // Optional momentary button to GND.
    // Gyro is detected at 0x68/0x69; install ONE MPU-6500/9250 or ICM-20948.
    constexpr int LeftMotorSign = 1; // Change to -1 if positive drives backwards.
    constexpr int RightMotorSign = 1;
    constexpr float GyroSign = 1.0f; // Positive yaw must mean turning LEFT.

    constexpr bool FloorUsesAnalog = false;
    constexpr int WhiteDigitalLevel = 0; // Measure this: it may need to be 1.
    constexpr int WhiteAnalogThreshold = 2000;
    constexpr bool WhiteAnalogIsHigher = true;
    // Conservative accepted range from the inventory; verify your actual ToFs.
    constexpr uint16_t MinimumRangeMm = 30;
    constexpr uint16_t MaximumRangeMm = 1200;

    // PWM commands, NOT measured speed in m/s. Tune for your motors and supply.
    constexpr int PwmFrequencyHz = 2000;
    constexpr int MaximumPwm = 160;
    constexpr int CruisePwm = 105;
    constexpr int SlowPwm = 85;
    constexpr int BenchPwm = 85;
    constexpr int MinimumTurnPwm = 75;
    constexpr int MaximumTurnPwm = 125;
    constexpr float WallGain = 0.50f; // PWM per mm of wall-distance error.
    constexpr float HeadingGain = 2.0f; // PWM per degree of yaw error.
    constexpr float TurnGain = 2.0f;
    constexpr int MaximumSteeringPwm = 30;

    // These are starting examples, NOT competition-ready dimensions.
    // Distances are from each sensor's optical face, not the chassis edge.
    constexpr float TargetWallMm = 45.0f;
    constexpr float SideOpeningMm = 170.0f;
    constexpr float WallPresentMm = 120.0f;
    constexpr float FrontDecisionMm = 130.0f;
    constexpr float FrontEmergencyMm = 70.0f;
    constexpr float SideEmergencyMm = 35.0f;
    constexpr float TurnEmergencyMm = 35.0f;
    constexpr float TurnToleranceDeg = 2.5f;
    constexpr float TurnSettledRateDps = 12.0f;

    // No installed wheel encoders are confirmed: junction translation is time-calibrated.
    // Calibrate with real corridor width, sensor offset and measured travel speed.
    constexpr uint32_t JunctionAdvanceMs = 350;
    constexpr uint32_t JunctionExitMs = 500;
    constexpr uint32_t TurnTimeoutMs = 5000;
    constexpr uint32_t TurnSettleMs = 100;
    constexpr uint32_t StartDelayMs = 3000;
    constexpr uint32_t RunTimeoutMs = 180000;
    constexpr uint32_t BlackConfirmMs = 200;
    constexpr uint32_t WhiteConfirmMs = 40;
    constexpr uint32_t SensorStaleMs = 180;
    constexpr uint32_t MaximumGyroGapMs = 100;
    constexpr uint32_t GyroCalibrationTimeoutMs = 3000;
    constexpr uint32_t BenchPulseMs = 300;
    constexpr uint32_t ButtonDebounceMs = 60;
    constexpr uint8_t OpeningConfirmFrames = 3;
    constexpr uint8_t FrontConfirmFrames = 2;
}
