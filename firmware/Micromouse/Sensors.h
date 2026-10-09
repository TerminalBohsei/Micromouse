// Stock: three VL53L0X/L1X breakouts, one MPU-6500/9250 or ICM-20948, KY-033.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <VL53L1X.h> // Pololu library, NOT Adafruit_VL53L1X.
#include "Config.h"
#include "Navigation.h"
#include "RegisterIO.h"
#include "GyroDriver.h"

struct RangeReading
{
    uint16_t mm = 0;
    uint8_t status = 255;
    uint32_t measuredMs = 0;
    bool valid = false;
};

enum class TofModel { Unknown, L0X, L1X };

class Sensors
{
public:
    Sensors() : mGyro(mRegisters) {}
    RangeReading range[3]; // Left, front, right.
    int floorRaw = 0;
    const char* error = "none";

    bool Begin()
    {
        pinMode(Config::FloorPin, Config::FloorUsesAnalog ? INPUT : INPUT_PULLUP);
        Wire.begin(Config::SdaPin, Config::SclPin);
        Wire.setClock(400000);
        Wire.setTimeOut(20);

        // Both ToF families start at 0x29. Assign addresses one at a time.
        const int xshut[] = {Config::LeftXshutPin, Config::FrontXshutPin, Config::RightXshutPin};
        for( int i = 0; i < 3; i++ )
        {
            digitalWrite(xshut[i], LOW);
            pinMode(xshut[i], OUTPUT);
        }
        delay(10);
        if( !mGyro.Begin() )
        {
            error = "gyro init failed: need ONE MPU-6500/9250 or ICM-20948 at 0x68/69";
            return false;
        }
        for( int i = 0; i < 3; i++ )
        {
            // Release, do not drive HIGH. Requires the breakout's XSHUT pull-up.
            pinMode(xshut[i], INPUT);
            delay(10);
            if( !BeginTof(i) )
            {
                Serial.printf("ToF initialization failed in slot %d (0=left,1=front,2=right)\n", i);
                error = "ToF init failed; check model, XSHUT, pull-up and wiring";
                return false;
            }
        }
        if( !CalibrateGyro() ) return false;
        mInitialized = true;
        return true;
    }

    void PrintModels() const
    {
        Serial.printf("Gyro: %s at 0x%02X\n", mGyro.Name(), mGyro.Address());
        for( int i = 0; i < 3; i++ )
            Serial.printf("ToF %d: %s at 0x%02X\n", i,
                          mTofModel[i] == TofModel::L0X ? "VL53L0X" :
                          (mTofModel[i] == TofModel::L1X ? "VL53L1X" : "unknown"), 0x30 + i);
    }

    // Call only while the motors are stopped and the chassis is stationary.
    bool CalibrateGyro()
    {
        float sum = 0.0f, lowest = 10000.0f, highest = -10000.0f;
        int count = 0;
        uint32_t startedMs = millis();
        while( count < 250 )
        {
            float rate = 0.0f;
            GyroSample sample = mGyro.ReadRate(rate);
            if( sample == GyroSample::Error || millis() - startedMs >= Config::GyroCalibrationTimeoutMs )
            {
                error = "gyro calibration read/timeout failed";
                mGyroFault = true;
                return false;
            }
            if( sample == GyroSample::Ready )
            {
                sum += rate;
                if( rate < lowest ) lowest = rate;
                if( rate > highest ) highest = rate;
                count++;
            }
            delay(1); // Only NEW sensor samples contribute to the calibration.
        }
        if( highest - lowest > 3.0f || fabsf(sum / count) > 15.0f )
        {
            error = "gyro calibration unstable; keep robot still";
            mGyroFault = true;
            return false;
        }
        mBiasDps = sum / count;
        mYawDeg = mRateDps = 0.0f;
        mGyroFault = false;
        mLastGyroSampleUs = mLastGyroPollUs = micros();
        error = "none";
        return true;
    }

    SensorFrame Read()
    {
        SensorFrame input;
        floorRaw = Config::FloorUsesAnalog ? analogRead(Config::FloorPin) : digitalRead(Config::FloorPin);
        input.whiteFloor = Config::FloorUsesAnalog
                          ? (Config::WhiteAnalogIsHigher ? floorRaw > Config::WhiteAnalogThreshold
                                                        : floorRaw < Config::WhiteAnalogThreshold)
                          : floorRaw == Config::WhiteDigitalLevel;
        if( mInitialized )
        {
            uint32_t nowUs = micros();
            if( nowUs - mLastGyroSampleUs > Config::MaximumGyroGapMs * 1000 ) mGyroFault = true;
            if( nowUs - mLastGyroPollUs >= 5000 )
            {
                mLastGyroPollUs = nowUs;
                float rawRate = 0.0f;
                GyroSample sample = mGyro.ReadRate(rawRate);
                if( sample == GyroSample::Error ) mGyroFault = true;
                else if( sample == GyroSample::Ready )
                {
                    mRateDps = Config::GyroSign * (rawRate - mBiasDps);
                    mYawDeg += mRateDps * ((nowUs - mLastGyroSampleUs) / 1000000.0f);
                    mLastGyroSampleUs = nowUs;
                }
            }
            // Asynchronous ranging: never wait for a measurement while moving.
            uint32_t pollMs = millis();
            if( pollMs - mLastRangePollMs >= 10 )
            {
                mLastRangePollMs = pollMs;
                for( int i = 0; i < 3; i++ ) PollTof(i);
                if( mNewRange[0] && mNewRange[1] && mNewRange[2] )
                {
                    mRangeSequence++;
                    for( int i = 0; i < 3; i++ ) mNewRange[i] = false;
                }
            }
        }
        input.nowMs = millis();
        input.rangeSequence = mRangeSequence;
        input.leftMm = range[0].mm;
        input.frontMm = range[1].mm;
        input.rightMm = range[2].mm;
        input.yawDeg = mYawDeg;
        input.yawRateDps = mRateDps;
        input.healthy = mInitialized && !mGyroFault;
        for( int i = 0; i < 3; i++ )
            input.healthy = input.healthy && range[i].valid &&
                            input.nowMs - range[i].measuredMs <= Config::SensorStaleMs;
        return input;
    }

private:
    RegisterIO mRegisters;
    GyroDriver<RegisterIO> mGyro;
    Adafruit_VL53L0X mL0[3];
    VL53L1X mL1[3];
    TofModel mTofModel[3] = {TofModel::Unknown, TofModel::Unknown, TofModel::Unknown};
    bool mNewRange[3] = {false, false, false};
    bool mInitialized = false;
    bool mGyroFault = false;
    uint32_t mRangeSequence = 0;
    uint32_t mLastRangePollMs = 0;
    uint32_t mLastGyroPollUs = 0;
    uint32_t mLastGyroSampleUs = 0;
    float mBiasDps = 0.0f;
    float mYawDeg = 0.0f;
    float mRateDps = 0.0f;

    bool BeginTof(int i)
    {
        uint8_t address = static_cast<uint8_t>(0x30 + i);
        // Probe L0X first. Sending a two-byte L1X register address to an L0X
        // would write its second byte into an L0X register.
        uint8_t l0Id = 0;
        if( mRegisters.Read(0x29, 0xC0, &l0Id, 1) && l0Id == 0xEE )
        {
            mTofModel[i] = TofModel::L0X;
            return mL0[i].begin(address, false, &Wire) && mL0[i].startRangeContinuous(50);
        }
        // Probe the L1X's 16-bit model ID before initializing either driver.
        Wire.beginTransmission(0x29);
        bool sent = Wire.write(static_cast<uint8_t>(0x01)) == 1 &&
                    Wire.write(static_cast<uint8_t>(0x0F)) == 1;
        if( Wire.endTransmission(false) != 0 || !sent ) return false;
        uint16_t id = 0;
        bool gotId = Wire.requestFrom(static_cast<uint8_t>(0x29), static_cast<uint8_t>(2)) == 2;
        if( gotId )
        {
            int high = Wire.read(), low = Wire.read();
            gotId = high >= 0 && low >= 0;
            if( gotId ) id = static_cast<uint16_t>((high << 8) | low);
        }
        else while( Wire.available() ) Wire.read();
        if( gotId && id == 0xEACC )
        {
            mTofModel[i] = TofModel::L1X;
            mL1[i].setBus(&Wire);
            mL1[i].setTimeout(100);
            if( !mL1[i].init() ) return false;
            mL1[i].setAddress(address);
            if( mL1[i].last_status != 0 || !mL1[i].setDistanceMode(VL53L1X::Short) ||
                !mL1[i].setMeasurementTimingBudget(20000) ) return false;
            mL1[i].startContinuous(50);
            return mL1[i].last_status == 0;
        }
        return false;
    }

    void PollTof(int i)
    {
        Wire.beginTransmission(static_cast<uint8_t>(0x30 + i));
        if( Wire.endTransmission() != 0 ) { Invalidate(i); return; }
        if( mTofModel[i] == TofModel::L0X )
        {
            bool ready = mL0[i].isRangeComplete();
            if( mL0[i].Status != VL53L0X_ERROR_NONE ) { Invalidate(i); return; }
            if( !ready ) return;
            range[i].mm = mL0[i].readRangeResult();
            range[i].status = mL0[i].readRangeStatus();
            if( mL0[i].Status != VL53L0X_ERROR_NONE ) { Invalidate(i); return; }
        }
        else if( mTofModel[i] == TofModel::L1X )
        {
            bool ready = mL1[i].dataReady();
            if( mL1[i].last_status != 0 ) { Invalidate(i); return; }
            if( !ready ) return;
            range[i].mm = mL1[i].read(false);
            range[i].status = static_cast<uint8_t>(mL1[i].ranging_data.range_status);
            if( mL1[i].last_status != 0 || mL1[i].timeoutOccurred() ) { Invalidate(i); return; }
        }
        else { Invalidate(i); return; }
        range[i].valid = range[i].status == 0 && range[i].mm >= Config::MinimumRangeMm &&
                        range[i].mm <= Config::MaximumRangeMm;
        range[i].measuredMs = millis();
        mNewRange[i] = true;
    }

    void Invalidate(int i)
    {
        range[i].valid = false;
        range[i].status = 255;
        mNewRange[i] = false;
    }
};
