// Hardware-independent wall follower. One right-hand controller is mirrored for LEFT mode.
#pragma once
#include <math.h>
#include <stdint.h>
#include "Config.h"

struct SensorFrame
{
    uint32_t nowMs = 0;
    uint32_t rangeSequence = 0; // Changes only after all three sensors update.
    float leftMm = 0.0f;
    float frontMm = 0.0f;
    float rightMm = 0.0f;
    float yawDeg = 0.0f;
    float yawRateDps = 0.0f;
    bool healthy = false;
    bool whiteFloor = false;
};

struct MotorCommand
{
    int left = 0;
    int right = 0;
    MotorCommand(int aLeft = 0, int aRight = 0) : left(aLeft), right(aRight) {}
};

enum class RobotState { Idle, Starting, Follow, ApproachJunction, Turn, ExitJunction, Finished, Fault };

class Navigator
{
public:
    explicit Navigator(bool aFollowLeftWall = Config::FollowLeftWall) : mFollowLeftWall(aFollowLeftWall) {}
    RobotState state = RobotState::Idle;
    const char* faultReason = "none";

    void Stop()
    {
        state = RobotState::Idle;
        faultReason = "none";
    }

    void Fault(const char* aReason)
    {
        state = RobotState::Fault;
        faultReason = aReason;
    }

    bool Start(const SensorFrame& aPhysicalInput)
    {
        SensorFrame aInput = Orient(aPhysicalInput);
        if( !Valid(aInput) ) return false;
        Reset(aInput);
        ChangeState(RobotState::Starting, aInput.nowMs);
        return true;
    }

    bool TurnTest(const SensorFrame& aPhysicalInput, float aAngleDeg)
    {
        SensorFrame aInput = Orient(aPhysicalInput);
        if( !Valid(aInput) ) return false;
        Reset(aInput);
        mTurnTest = true;
        // Serial test angles keep their physical meaning in either wall mode.
        BeginTurn(aInput, mFollowLeftWall ? -aAngleDeg : aAngleDeg);
        return true;
    }

    MotorCommand Update(const SensorFrame& aPhysicalInput)
    {
        MotorCommand command = UpdateCanonical(Orient(aPhysicalInput));
        return mFollowLeftWall ? MotorCommand(command.right, command.left) : command;
    }

private:
    MotorCommand UpdateCanonical(const SensorFrame& aInput)
    {
        if( state == RobotState::Idle || state == RobotState::Finished || state == RobotState::Fault )
            return MotorCommand();

        if( !Valid(aInput) )
        {
            Fault("invalid or stale sensor data");
            return MotorCommand();
        }
        if( aInput.nowMs - mRunStartedMs > Config::RunTimeoutMs )
        {
            Fault("run timeout; possible maze loop");
            return MotorCommand();
        }
        if( state != RobotState::Starting && FinishDetected(aInput) )
        {
            state = RobotState::Finished;
            return MotorCommand();
        }

        UpdateEvidence(aInput);

        if( state == RobotState::Starting )
        {
            if( aInput.nowMs - mStateStartedMs >= Config::StartDelayMs )
            {
                mHeadingDeg = aInput.yawDeg;
                ClearEvidence(aInput.rangeSequence);
                ChangeState(RobotState::Follow, aInput.nowMs);
            }
            return MotorCommand();
        }

        if( state == RobotState::Turn ) return Turn(aInput);

        // Translation stops before the robot gets too close. This does NOT
        // guarantee corner clearance: physically check the full swept chassis.
        if( aInput.frontMm <= Config::FrontEmergencyMm ||
            aInput.leftMm <= Config::SideEmergencyMm ||
            aInput.rightMm <= Config::SideEmergencyMm )
        {
            Fault("translation clearance limit");
            return MotorCommand();
        }

        if( state == RobotState::ApproachJunction )
        {
            if( aInput.frontMm <= Config::FrontDecisionMm ||
                aInput.nowMs - mStateStartedMs >= Config::JunctionAdvanceMs )
            {
                BeginTurn(aInput, -90.0f);
                return MotorCommand();
            }
            return Straight(aInput, Config::SlowPwm, false);
        }

        if( state == RobotState::ExitJunction )
        {
            if( aInput.frontMm <= Config::FrontDecisionMm ||
                aInput.nowMs - mStateStartedMs >= Config::JunctionExitMs )
            {
                ClearEvidence(aInput.rangeSequence);
                ChangeState(RobotState::Follow, aInput.nowMs);
                return MotorCommand();
            }
            return Straight(aInput, Config::SlowPwm, false);
        }

        // Junction priority: right, straight, left, back. No stored route.
        if( mRightOpenFrames >= Config::OpeningConfirmFrames && aInput.rightMm > Config::SideOpeningMm )
        {
            mHeadingDeg = aInput.yawDeg;
            if( aInput.frontMm > Config::FrontDecisionMm && Config::JunctionAdvanceMs > 0 )
                ChangeState(RobotState::ApproachJunction, aInput.nowMs);
            else
                BeginTurn(aInput, -90.0f);
            return MotorCommand();
        }

        if( aInput.frontMm <= Config::FrontDecisionMm )
        {
            // Stop while gathering fresh evidence, rather than entering a wall.
            if( mFrontBlockedFrames < Config::FrontConfirmFrames ) return MotorCommand();
            if( aInput.rightMm > Config::SideOpeningMm ) return MotorCommand();
            if( aInput.leftMm > Config::SideOpeningMm )
            {
                if( mLeftOpenFrames < Config::OpeningConfirmFrames ) return MotorCommand();
                BeginTurn(aInput, 90.0f);
            }
            else
                BeginTurn(aInput, 180.0f);
            return MotorCommand();
        }

        // Slow down near an obstacle or an unconfirmed right-hand opening.
        int speed = (aInput.frontMm < 2.0f * Config::FrontDecisionMm || mRightOpenFrames > 0)
                    ? Config::SlowPwm : Config::CruisePwm;
        return Straight(aInput, speed, true);
    }

public:
    static const char* StateName(RobotState aState)
    {
        switch( aState )
        {
            case RobotState::Idle: return "IDLE";
            case RobotState::Starting: return "STARTING";
            case RobotState::Follow: return "FOLLOW";
            case RobotState::ApproachJunction: return "APPROACH_JUNCTION";
            case RobotState::Turn: return "TURN";
            case RobotState::ExitJunction: return "EXIT_JUNCTION";
            case RobotState::Finished: return "FINISHED";
            case RobotState::Fault: return "FAULT";
        }
        return "UNKNOWN";
    }

private:
    const bool mFollowLeftWall;
    uint32_t mRunStartedMs = 0;
    uint32_t mStateStartedMs = 0;
    uint32_t mLastSequence = 0;
    uint32_t mBlackStartedMs = 0;
    uint32_t mWhiteStartedMs = 0;
    uint32_t mSettledStartedMs = 0;
    uint8_t mRightOpenFrames = 0;
    uint8_t mLeftOpenFrames = 0;
    uint8_t mFrontBlockedFrames = 0;
    float mHeadingDeg = 0.0f;
    float mTurnTargetDeg = 0.0f;
    bool mBlackTiming = false;
    bool mWhiteTiming = false;
    bool mBlackSeen = false;
    bool mSettledTiming = false;
    bool mTurnTest = false;

    SensorFrame Orient(SensorFrame aInput) const
    {
        if( mFollowLeftWall )
        {
            float left = aInput.leftMm;
            aInput.leftMm = aInput.rightMm;
            aInput.rightMm = left;
            aInput.yawDeg = -aInput.yawDeg;
            aInput.yawRateDps = -aInput.yawRateDps;
        }
        return aInput;
    }

    static bool Valid(const SensorFrame& aInput)
    {
        return aInput.healthy && isfinite(aInput.yawDeg) && isfinite(aInput.yawRateDps) &&
               isfinite(aInput.leftMm) && isfinite(aInput.frontMm) && isfinite(aInput.rightMm) &&
               aInput.leftMm > 0.0f && aInput.frontMm > 0.0f && aInput.rightMm > 0.0f;
    }

    static int Clamp(int aValue, int aMin, int aMax)
    {
        return aValue < aMin ? aMin : (aValue > aMax ? aMax : aValue);
    }

    void Reset(const SensorFrame& aInput)
    {
        faultReason = "none";
        mRunStartedMs = aInput.nowMs;
        mHeadingDeg = aInput.yawDeg;
        mBlackTiming = mWhiteTiming = mBlackSeen = mSettledTiming = mTurnTest = false;
        ClearEvidence(aInput.rangeSequence);
    }

    void ChangeState(RobotState aState, uint32_t aNowMs)
    {
        state = aState;
        mStateStartedMs = aNowMs;
    }

    void ClearEvidence(uint32_t aSequence)
    {
        mLastSequence = aSequence;
        mRightOpenFrames = mLeftOpenFrames = mFrontBlockedFrames = 0;
    }

    void UpdateEvidence(const SensorFrame& aInput)
    {
        if( aInput.rangeSequence == mLastSequence ) return;
        mLastSequence = aInput.rangeSequence;
        mRightOpenFrames = aInput.rightMm > Config::SideOpeningMm
                           ? static_cast<uint8_t>(Clamp(mRightOpenFrames + 1, 0, 10)) : 0;
        mLeftOpenFrames = aInput.leftMm > Config::SideOpeningMm
                          ? static_cast<uint8_t>(Clamp(mLeftOpenFrames + 1, 0, 10)) : 0;
        mFrontBlockedFrames = aInput.frontMm <= Config::FrontDecisionMm
                              ? static_cast<uint8_t>(Clamp(mFrontBlockedFrames + 1, 0, 10)) : 0;
    }

    bool FinishDetected(const SensorFrame& aInput)
    {
        if( !aInput.whiteFloor )
        {
            mWhiteTiming = false;
            if( !mBlackTiming ) { mBlackTiming = true; mBlackStartedMs = aInput.nowMs; }
            if( aInput.nowMs - mBlackStartedMs >= Config::BlackConfirmMs ) mBlackSeen = true;
            return false;
        }
        mBlackTiming = false;
        if( !mBlackSeen ) return false;
        if( !mWhiteTiming ) { mWhiteTiming = true; mWhiteStartedMs = aInput.nowMs; }
        return aInput.nowMs - mWhiteStartedMs >= Config::WhiteConfirmMs;
    }

    MotorCommand Straight(const SensorFrame& aInput, int aSpeed, bool aUseWall)
    {
        // Positive correction makes the right wheel faster, steering LEFT.
        float correction = Config::HeadingGain * (mHeadingDeg - aInput.yawDeg);
        if( aUseWall && aInput.rightMm <= Config::WallPresentMm )
            correction += Config::WallGain * (Config::TargetWallMm - aInput.rightMm);
        int steer = Clamp(static_cast<int>(correction), -Config::MaximumSteeringPwm,
                          Config::MaximumSteeringPwm);
        return MotorCommand(Clamp(aSpeed - steer, 0, Config::MaximumPwm),
                            Clamp(aSpeed + steer, 0, Config::MaximumPwm));
    }

    void BeginTurn(const SensorFrame& aInput, float aAngleDeg)
    {
        mTurnTargetDeg = aInput.yawDeg + aAngleDeg;
        mSettledTiming = false;
        ChangeState(RobotState::Turn, aInput.nowMs);
    }

    MotorCommand Turn(const SensorFrame& aInput)
    {
        if( aInput.nowMs - mStateStartedMs > Config::TurnTimeoutMs )
        {
            Fault("turn timeout; check motor direction and gyro sign");
            return MotorCommand();
        }
        if( aInput.frontMm <= Config::TurnEmergencyMm || aInput.leftMm <= Config::TurnEmergencyMm ||
            aInput.rightMm <= Config::TurnEmergencyMm )
        {
            Fault("turn clearance limit");
            return MotorCommand();
        }
        float error = mTurnTargetDeg - aInput.yawDeg;
        if( fabsf(error) <= Config::TurnToleranceDeg )
        {
            if( fabsf(aInput.yawRateDps) > Config::TurnSettledRateDps )
            {
                mSettledTiming = false;
                return MotorCommand();
            }
            if( !mSettledTiming ) { mSettledTiming = true; mSettledStartedMs = aInput.nowMs; }
            if( aInput.nowMs - mSettledStartedMs >= Config::TurnSettleMs )
            {
                mHeadingDeg = mTurnTargetDeg;
                if( mTurnTest ) Stop();
                else ChangeState(RobotState::ExitJunction, aInput.nowMs);
            }
            return MotorCommand();
        }
        mSettledTiming = false;
        int pwm = Clamp(static_cast<int>(Config::TurnGain * fabsf(error)),
                        Config::MinimumTurnPwm, Config::MaximumTurnPwm);
        return error > 0.0f ? MotorCommand(-pwm, pwm) : MotorCommand(pwm, -pwm);
    }
};
