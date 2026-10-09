// L298N differential-drive adapter. ENA/ENB jumpers must be REMOVED.
#pragma once
#include <Arduino.h>
#include <esp_arduino_version.h>
#include "Config.h"
#include "Navigation.h"

class Motors
{
public:
    bool Begin()
    {
        const int pins[] = {Config::LeftEnablePin, Config::RightEnablePin, Config::LeftIn1Pin,
                            Config::LeftIn2Pin, Config::RightIn1Pin, Config::RightIn2Pin};
        for( int i = 0; i < 6; i++ )
        {
            digitalWrite(pins[i], LOW);
            pinMode(pins[i], OUTPUT);
        }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        bool leftOk = ledcAttach(Config::LeftEnablePin, Config::PwmFrequencyHz, 8);
        bool rightOk = ledcAttach(Config::RightEnablePin, Config::PwmFrequencyHz, 8);
        mReady = leftOk && rightOk;
#else
        double leftHz = ledcSetup(0, Config::PwmFrequencyHz, 8);
        double rightHz = ledcSetup(1, Config::PwmFrequencyHz, 8);
        ledcAttachPin(Config::LeftEnablePin, 0);
        ledcAttachPin(Config::RightEnablePin, 1);
        mReady = leftHz > 0.0 && rightHz > 0.0;
#endif
        Set(MotorCommand());
        return mReady;
    }

    void Set(MotorCommand aCommand)
    {
        if( !Config::AllowMotorMotion || !mReady ) aCommand = MotorCommand();
        SetOne(aCommand.left * Config::LeftMotorSign, Config::LeftEnablePin,
               Config::LeftIn1Pin, Config::LeftIn2Pin, 0);
        SetOne(aCommand.right * Config::RightMotorSign, Config::RightEnablePin,
               Config::RightIn1Pin, Config::RightIn2Pin, 1);
    }

private:
    bool mReady = false;

    void Duty(int aPin, int aChannel, int aDuty)
    {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        (void)aChannel;
        ledcWrite(aPin, aDuty);
#else
        (void)aPin;
        ledcWrite(aChannel, aDuty);
#endif
    }

    void SetOne(int aCommand, int aEnable, int aIn1, int aIn2, int aChannel)
    {
        int duty = abs(aCommand);
        if( duty > Config::MaximumPwm ) duty = Config::MaximumPwm;
        // Disable the bridge while changing direction; zero duty gives coasting.
        Duty(aEnable, aChannel, 0);
        digitalWrite(aIn1, aCommand > 0 ? HIGH : LOW);
        digitalWrite(aIn2, aCommand < 0 ? HIGH : LOW);
        Duty(aEnable, aChannel, duty);
    }
};
