// Tests the decision/state logic only. No Arduino, motors or sensors are needed.
#include <iostream>
#include <stdexcept>
#include <limits>
#include "../firmware/Micromouse/Navigation.h"

int checks = 0;

void Check(bool condition, const char* message)
{
    checks++;
    if( !condition ) throw std::runtime_error(message);
}

SensorFrame Frame(uint32_t now = 0)
{
    SensorFrame input;
    input.nowMs = now;
    input.leftMm = 70;
    input.frontMm = 500;
    input.rightMm = Config::TargetWallMm;
    input.healthy = true;
    return input;
}

SensorFrame Activate(Navigator& nav, uint32_t start = 0)
{
    SensorFrame input = Frame(start);
    Check(nav.Start(input), "healthy start refused");
    MotorCommand cmd = nav.Update(input);
    Check(cmd.left == 0 && cmd.right == 0, "start delay moved motors");
    input.nowMs += Config::StartDelayMs;
    nav.Update(input);
    Check(nav.state == RobotState::Follow, "start delay did not enter FOLLOW");
    input.nowMs += 10;
    return input;
}

MotorCommand Fresh(Navigator& nav, SensorFrame& input)
{
    input.nowMs += 50;
    input.rangeSequence++;
    return nav.Update(input);
}

void WallAndHeadingTests()
{
    Navigator nav(false);
    SensorFrame input = Activate(nav);
    MotorCommand cmd = Fresh(nav, input);
    Check(cmd.left == cmd.right && cmd.left > 0, "straight corridor not straight");
    input.rightMm = 40;
    cmd = Fresh(nav, input);
    Check(cmd.right > cmd.left, "too close to right wall did not steer LEFT");
    input.rightMm = 90;
    cmd = Fresh(nav, input);
    Check(cmd.left > cmd.right, "too far from right wall did not steer RIGHT");
    input.rightMm = Config::TargetWallMm;
    input.yawDeg = 5;
    cmd = Fresh(nav, input);
    Check(cmd.left > cmd.right, "positive yaw error did not steer RIGHT");
    input.yawDeg = -5;
    cmd = Fresh(nav, input);
    Check(cmd.right > cmd.left, "negative yaw error did not steer LEFT");
    input.leftMm = 600;
    input.yawDeg = 0;
    for( int i = 0; i < 4; i++ ) Fresh(nav, input);
    Check(nav.state == RobotState::Follow, "left opening overrode clear forward path");
}

void RightJunctionTests()
{
    Navigator nav(false);
    SensorFrame input = Activate(nav);
    input.rightMm = 500;
    Fresh(nav, input);
    for( int i = 0; i < 100; i++ ) { input.nowMs++; nav.Update(input); }
    Check(nav.state == RobotState::Follow, "same ToF frame counted as many readings");
    Fresh(nav, input);
    Check(nav.state == RobotState::Follow, "opening accepted before confirmation");
    Fresh(nav, input);
    Check(nav.state == RobotState::ApproachJunction, "confirmed right opening not prioritised");
    input.nowMs += Config::JunctionAdvanceMs;
    nav.Update(input);
    Check(nav.state == RobotState::Turn, "junction approach did not enter TURN");
    MotorCommand cmd = nav.Update(input);
    Check(cmd.left > 0 && cmd.right < 0, "right turn wheel signs reversed");
    input.yawDeg = -95;
    cmd = nav.Update(input);
    Check(cmd.left < 0 && cmd.right > 0, "turn overshoot not corrected");
    input.yawDeg = -90;
    input.yawRateDps = 50;
    cmd = nav.Update(input);
    Check(cmd.left == 0 && cmd.right == 0 && nav.state == RobotState::Turn,
          "turn completed while rotating fast");
    input.yawRateDps = 0;
    nav.Update(input);
    input.nowMs += Config::TurnSettleMs;
    nav.Update(input);
    Check(nav.state == RobotState::ExitJunction, "settled right turn not completed");
    input.rightMm = Config::TargetWallMm;
    input.nowMs += Config::JunctionExitMs;
    nav.Update(input);
    Check(nav.state == RobotState::Follow, "junction exit did not resume following");
    cmd = Fresh(nav, input);
    Check(cmd.left > 0 && cmd.right > 0, "did not drive after completing right turn");

    Navigator noisy(false);
    input = Activate(noisy);
    input.rightMm = 500;
    Fresh(noisy, input);
    input.rightMm = Config::TargetWallMm;
    Fresh(noisy, input);
    input.rightMm = 500;
    Fresh(noisy, input);
    Fresh(noisy, input);
    Check(noisy.state == RobotState::Follow, "single opening outlier was not reset");
}

void JunctionPriorityTests()
{
    Navigator nav(false);
    SensorFrame input = Activate(nav);
    input.frontMm = 100;
    input.leftMm = 500;
    input.rightMm = 500;
    for( int i = 0; i < 3; i++ ) Fresh(nav, input);
    MotorCommand cmd = nav.Update(input);
    Check(nav.state == RobotState::Turn && cmd.left > 0 && cmd.right < 0,
          "right did not beat left at blocked junction");

    Navigator left(false);
    input = Activate(left);
    input.frontMm = 100;
    input.leftMm = 500;
    for( int i = 0; i < 3; i++ ) Fresh(left, input);
    cmd = left.Update(input);
    Check(left.state == RobotState::Turn && cmd.left < 0 && cmd.right > 0,
          "left-only corner not handled");

    Navigator dead(false);
    input = Activate(dead);
    input.frontMm = 100;
    Fresh(dead, input);
    Fresh(dead, input);
    cmd = dead.Update(input);
    Check(dead.state == RobotState::Turn && cmd.left < 0 && cmd.right > 0,
          "dead end did not start a U-turn");
    input.yawDeg = 90;
    cmd = dead.Update(input);
    Check(cmd.left < 0 && cmd.right > 0, "U-turn stopped at 90 instead of 180 degrees");
    input.yawDeg = 180;
    dead.Update(input);
    input.nowMs += Config::TurnSettleMs;
    dead.Update(input);
    Check(dead.state == RobotState::ExitJunction, "U-turn did not finish at 180 degrees");
}

void StopAndFaultTests()
{
    Navigator idle(false);
    SensorFrame input = Frame();
    MotorCommand cmd = idle.Update(input);
    Check(cmd.left == 0 && cmd.right == 0, "idle did not stop");
    input.healthy = false;
    Check(!idle.Start(input), "unhealthy start accepted");

    Navigator bad(false);
    input = Activate(bad);
    input.healthy = false;
    cmd = bad.Update(input);
    Check(bad.state == RobotState::Fault && cmd.left == 0 && cmd.right == 0,
          "sensor failure did not latch a stop");
    input.healthy = true;
    bad.Update(input);
    Check(bad.state == RobotState::Fault, "fault restarted automatically");

    Navigator close(false);
    input = Activate(close);
    input.frontMm = Config::FrontEmergencyMm;
    cmd = close.Update(input);
    Check(close.state == RobotState::Fault && cmd.left == 0, "front clearance safety failed");

    Navigator side(false);
    input = Activate(side);
    input.rightMm = Config::SideEmergencyMm;
    side.Update(input);
    Check(side.state == RobotState::Fault, "side clearance safety failed");

    Navigator invalid(false);
    input = Activate(invalid);
    input.rightMm = 0;
    invalid.Update(input);
    Check(invalid.state == RobotState::Fault, "zero range treated as an opening");

    Navigator nan(false);
    input = Activate(nan);
    input.yawDeg = std::numeric_limits<float>::quiet_NaN();
    nan.Update(input);
    Check(nan.state == RobotState::Fault, "NaN yaw accepted");

    Navigator timeout(false);
    input = Frame();
    timeout.TurnTest(input, -90);
    input.nowMs = Config::TurnTimeoutMs + 1;
    cmd = timeout.Update(input);
    Check(timeout.state == RobotState::Fault && cmd.left == 0, "turn timeout failed");

    Navigator runTimeout(false);
    input = Activate(runTimeout);
    input.nowMs = Config::RunTimeoutMs + 1;
    runTimeout.Update(input);
    Check(runTimeout.state == RobotState::Fault, "run timeout failed");
}

void FinishTests()
{
    Navigator nav(false);
    SensorFrame input = Activate(nav);
    input.whiteFloor = true;
    Fresh(nav, input);
    input.nowMs += 500;
    nav.Update(input);
    Check(nav.state != RobotState::Finished, "white at startup accepted without seeing black");
    input.whiteFloor = false;
    nav.Update(input);
    input.nowMs += Config::BlackConfirmMs;
    nav.Update(input);
    input.whiteFloor = true;
    input.nowMs += 10;
    nav.Update(input);
    input.nowMs += Config::WhiteConfirmMs - 1;
    nav.Update(input);
    Check(nav.state != RobotState::Finished, "finish accepted too early");
    input.whiteFloor = false;
    nav.Update(input);
    input.whiteFloor = true;
    input.nowMs += 1;
    nav.Update(input);
    input.nowMs += Config::WhiteConfirmMs;
    MotorCommand cmd = nav.Update(input);
    Check(nav.state == RobotState::Finished && cmd.left == 0 && cmd.right == 0,
          "sustained finish did not stop motors");
    input.whiteFloor = false;
    cmd = nav.Update(input);
    Check(nav.state == RobotState::Finished && cmd.left == 0, "finish was not latched");
}

void TurnTestAndClockTests()
{
    Navigator nav(false);
    SensorFrame input = Frame();
    Check(nav.TurnTest(input, 90), "turn test did not start");
    input.yawDeg = 90;
    nav.Update(input);
    input.nowMs += Config::TurnSettleMs;
    nav.Update(input);
    Check(nav.state == RobotState::Idle, "turn test drove forward after turning");

    Navigator rollover(false);
    input = Activate(rollover, UINT32_MAX - 1000);
    MotorCommand cmd = Fresh(rollover, input);
    Check(rollover.state == RobotState::Follow && cmd.left > 0,
          "millis rollover broke startup/following");
    rollover.Stop();
    cmd = rollover.Update(input);
    Check(rollover.state == RobotState::Idle && cmd.left == 0 && cmd.right == 0,
          "explicit stop failed");
}

int main()
{
    try
    {
        WallAndHeadingTests();
        RightJunctionTests();
        JunctionPriorityTests();
        StopAndFaultTests();
        FinishTests();
        TurnTestAndClockTests();
        std::cout << "PASS: " << checks << " navigation checks\n";
        return 0;
    }
    catch( const std::exception& error )
    {
        std::cerr << "FAIL after check " << checks << ": " << error.what() << '\n';
        return 1;
    }
}
