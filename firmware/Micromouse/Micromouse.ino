// Continuation of Kohsei's setup(), printReading() and loop() sketch.
// Config.h holds GPIO numbers; Motors.h and Sensors.h initialize the hardware.
// Default upload prints readings with motor motion disabled.
#include "Config.h"
#include "Motors.h"
#include "Sensors.h"
#include "Navigation.h"

Motors motors;
Sensors sensors;
Navigator navigator;
bool sensorsReady = false;
bool motorsReady = false;
bool benchActive = false;
uint32_t benchStartedMs = 0;
uint32_t lastPrintMs = 0;

void StopRobot()
{
    benchActive = false;
    navigator.Stop();
    motors.Set(MotorCommand());
}

void PrintHelp()
{
    Serial.println(Config::FollowLeftWall ? "Wall strategy: LEFT" : "Wall strategy: RIGHT");
    Serial.println("H help | Q stop | C calibrate gyro (keep still)");
    Serial.println("F/B: 300 ms forward/back BENCH pulse, wheels lifted");
    Serial.println("T/Y/U: gyro turn test right 90 / left 90 / left 180");
    Serial.println("S: start wall following after 3 seconds; optional GPIO4 button");
    Serial.println(Config::AllowMotorMotion ? "Motor motion enabled; calibration still required for S"
                                           : "Motor motion DISABLED. Verify Config.h and START_HERE.md.");
}

void RequestStart(SensorFrame input)
{
    StopRobot();
    if( !Config::AllowMotorMotion || !Config::NavigationCalibrated )
    {
        Serial.println("Start refused: complete wiring/calibration checks in Config.h.");
        return;
    }
    if( !sensorsReady || !motorsReady || !input.healthy )
    {
        Serial.println("Start refused: sensors/motors not ready.");
        return;
    }
    // C performs stationary calibration separately; starting never hides a long delay.
    navigator.Start(input);
    Serial.println("Starting in 3 seconds. Q or the power switch stops the robot.");
}

void HandleCommand(char command, SensorFrame input)
{
    if( command >= 'a' && command <= 'z' ) command = command - 'a' + 'A';
    if( command == 'Q' ) { StopRobot(); Serial.println("Stopped"); }
    else if( command == 'H' ) PrintHelp();
    else if( command == 'C' )
    {
        StopRobot();
        if( !sensorsReady ) { Serial.println("Sensor init failed; correct wiring and restart."); return; }
        Serial.println("Keep the robot stationary: calibrating gyro.");
        bool ok = sensors.CalibrateGyro();
        Serial.println(ok ? "Gyro calibrated" : sensors.error);
    }
    else if( command == 'S' ) RequestStart(input);
    else if( command == 'F' || command == 'B' )
    {
        StopRobot();
        if( !Config::AllowMotorMotion || !motorsReady ) { Serial.println("Motor motion disabled"); return; }
        Serial.println("BENCH ONLY: wheels must be lifted. Automatic stop in 300 ms.");
        int pwm = command == 'F' ? Config::BenchPwm : -Config::BenchPwm;
        benchStartedMs = millis();
        benchActive = true;
        motors.Set(MotorCommand(pwm, pwm));
    }
    else if( command == 'T' || command == 'Y' || command == 'U' )
    {
        StopRobot();
        if( !Config::AllowMotorMotion || !motorsReady || !sensorsReady || !input.healthy )
        { Serial.println("Turn refused: motor/sensor checks not complete"); return; }
        float angle = command == 'T' ? -90.0f : (command == 'Y' ? 90.0f : 180.0f);
        navigator.TurnTest(input, angle);
    }
}

void CheckStartButton(const SensorFrame& input)
{
    static bool timing = false;
    static bool handled = false;
    static uint32_t pressedMs = 0;
    if( digitalRead(Config::StartPin) == HIGH ) { timing = handled = false; return; }
    if( !timing ) { timing = true; pressedMs = input.nowMs; }
    if( !handled && input.nowMs - pressedMs >= Config::ButtonDebounceMs )
    {
        handled = true;
        // A button press during a run stops it; only an idle bot can be started.
        if( navigator.state == RobotState::Idle && !benchActive ) RequestStart(input);
        else StopRobot();
    }
}

void printReading(const SensorFrame& input)
{
    Serial.printf("%s L=%u(%u) F=%u(%u) R=%u(%u) yaw=%.1f rate=%.1f IR=%d white=%d ok=%d\n",
                  Navigator::StateName(navigator.state), sensors.range[0].mm, sensors.range[0].status,
                  sensors.range[1].mm, sensors.range[1].status, sensors.range[2].mm, sensors.range[2].status,
                  input.yawDeg, input.yawRateDps, sensors.floorRaw, input.whiteFloor, input.healthy);
    if( navigator.state == RobotState::Fault ) Serial.println(navigator.faultReason);
}

void setup()
{
    Serial.begin(115200);
    pinMode(Config::StartPin, INPUT_PULLUP);
    // Sets the six L298N control pins, PWM outputs and a stopped command.
    motorsReady = motors.Begin();
    Serial.println("Keep robot stationary during startup gyro calibration.");
    // Starts I2C, gives each ToF its own address, and calibrates the gyro.
    sensorsReady = sensors.Begin();
    if( !sensorsReady ) Serial.println(sensors.error);
    sensors.PrintModels();
    if( !motorsReady ) Serial.println("PWM initialization failed");
    PrintHelp();
}

void loop()
{
    // Physical L/F/R distances are millimetres, not centimetres or pulse times.
    SensorFrame input = sensors.Read();
    // One byte per iteration avoids using old sensor data after a blocking command.
    if( Serial.available() > 0 )
    {
        HandleCommand(static_cast<char>(Serial.read()), input);
        input = sensors.Read();
    }
    CheckStartButton(input);
    if( benchActive )
    {
        if( millis() - benchStartedMs >= Config::BenchPulseMs ) StopRobot();
    }
    else
        // Read sensors -> choose a motion state -> apply left/right motor PWM.
        motors.Set(navigator.Update(input));

    if( input.nowMs - lastPrintMs >= 200 )
    {
        lastPrintMs = input.nowMs;
        printReading(input);
    }
    // No delay(100): steering and stop checks continue between printouts.
}
