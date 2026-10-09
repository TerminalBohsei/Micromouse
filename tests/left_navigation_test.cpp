// Physical LEFT-wall expectations, independent of the right-wall test fixtures.
#include <iostream>
#include <stdexcept>
#include "../firmware/Micromouse/Navigation.h"

int checks = 0;
void Check(bool condition, const char* message)
{
    checks++;
    if( !condition ) throw std::runtime_error(message);
}

SensorFrame Frame()
{
    SensorFrame input;
    input.healthy = true;
    input.leftMm = Config::TargetWallMm;
    input.frontMm = 500;
    input.rightMm = 70;
    input.yawDeg = 30; // Nonzero initial heading catches incorrect frame handling.
    return input;
}

SensorFrame Activate(Navigator& nav)
{
    SensorFrame input = Frame();
    Check(nav.Start(input), "left-mode start failed");
    MotorCommand command = nav.Update(input);
    Check(command.left == 0 && command.right == 0, "left mode bypassed start delay");
    input.nowMs += Config::StartDelayMs;
    nav.Update(input);
    Check(nav.state == RobotState::Follow, "left mode did not enter FOLLOW");
    return input;
}

MotorCommand Fresh(Navigator& nav, SensorFrame& input)
{
    input.nowMs += 50;
    input.rangeSequence++;
    return nav.Update(input);
}

int main()
{
    Navigator wall(true);
    SensorFrame input = Activate(wall);
    MotorCommand command = Fresh(wall, input);
    Check(command.left == command.right && command.left > 0, "left corridor not straight");
    input.leftMm = 40;
    command = Fresh(wall, input);
    Check(command.left > command.right, "close LEFT wall did not steer RIGHT");
    input.leftMm = 90;
    command = Fresh(wall, input);
    Check(command.right > command.left, "distant LEFT wall did not steer LEFT");
    input.leftMm = Config::TargetWallMm;
    input.yawDeg = 35;
    command = Fresh(wall, input);
    Check(command.left > command.right, "left mode did not correct positive yaw to the RIGHT");
    input.yawDeg = 30;
    input.rightMm = 500;
    for( int i = 0; i < 3; i++ ) Fresh(wall, input);
    Check(wall.state == RobotState::Follow, "right opening overrode forward in LEFT mode");
    input.leftMm = 500;
    for( int i = 0; i < 3; i++ ) Fresh(wall, input);
    Check(wall.state == RobotState::ApproachJunction, "left opening was not prioritized");
    input.nowMs += Config::JunctionAdvanceMs;
    wall.Update(input);
    command = wall.Update(input);
    Check(wall.state == RobotState::Turn && command.left < 0 && command.right > 0,
          "LEFT mode junction turned right");
    input.yawDeg = 125; // Overshoot target: 30 + 90 = 120.
    command = wall.Update(input);
    Check(command.left > 0 && command.right < 0, "left turn overshoot not corrected right");
    input.yawDeg = 120;
    input.yawRateDps = -50;
    wall.Update(input);
    input.nowMs += Config::TurnSettleMs;
    wall.Update(input);
    Check(wall.state == RobotState::Turn, "left turn finished while yaw rate was high");
    input.yawRateDps = 0;
    wall.Update(input);
    input.nowMs += Config::TurnSettleMs;
    wall.Update(input);
    Check(wall.state == RobotState::ExitJunction, "left turn target yaw was wrong");

    Navigator blocked(true);
    input = Activate(blocked);
    input.frontMm = 100;
    input.rightMm = 500;
    for( int i = 0; i < 3; i++ ) Fresh(blocked, input);
    command = blocked.Update(input);
    Check(blocked.state == RobotState::Turn && command.left > 0 && command.right < 0,
          "left-closed, front-blocked junction did not turn RIGHT");

    Navigator dead(true);
    input = Activate(dead);
    input.frontMm = 100;
    for( int i = 0; i < 3; i++ ) Fresh(dead, input);
    command = dead.Update(input);
    Check(dead.state == RobotState::Turn && command.left > 0 && command.right < 0,
          "left-mode dead end did not start the mirrored U-turn");
    input.yawDeg = -150;
    dead.Update(input);
    input.nowMs += Config::TurnSettleMs;
    dead.Update(input);
    Check(dead.state == RobotState::ExitJunction, "left-mode U-turn did not reach 180 degrees");

    Navigator test(true);
    input = Frame();
    Check(test.TurnTest(input, -90), "physical right turn test failed to start in LEFT mode");
    command = test.Update(input);
    Check(command.left > 0 && command.right < 0, "T test changed physical meaning in LEFT mode");
    input.yawDeg = -60;
    test.Update(input);
    input.nowMs += Config::TurnSettleMs;
    test.Update(input);
    Check(test.state == RobotState::Idle, "physical right turn test did not finish correctly");

    Navigator finish(true);
    input = Activate(finish);
    finish.Update(input);
    input.nowMs += Config::BlackConfirmMs;
    finish.Update(input);
    input.whiteFloor = true;
    finish.Update(input);
    input.nowMs += Config::WhiteConfirmMs;
    command = finish.Update(input);
    Check(finish.state == RobotState::Finished && command.left == 0 && command.right == 0,
          "left mode did not stop at finish");

    Navigator fault(true);
    input = Activate(fault);
    input.leftMm = Config::SideEmergencyMm;
    command = fault.Update(input);
    Check(fault.state == RobotState::Fault && command.left == 0 && command.right == 0,
          "left-mode clearance limit did not stop both motors");
    std::cout << "PASS: " << checks << " left-navigation checks\n";
}
