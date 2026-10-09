// Checked I2C transfers: an ACK alone does not prove a complete sensor reading.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class RegisterIO
{
public:
    bool Read(uint8_t address, uint8_t reg, uint8_t* data, uint8_t count)
    {
        Wire.beginTransmission(address);
        if( Wire.write(reg) != 1 || Wire.endTransmission(false) != 0 ) return false;
        if( Wire.requestFrom(address, count) != count )
        {
            while( Wire.available() ) Wire.read();
            return false;
        }
        for( uint8_t i = 0; i < count; i++ )
        {
            if( !Wire.available() ) return false;
            int value = Wire.read();
            if( value < 0 ) return false;
            data[i] = static_cast<uint8_t>(value);
        }
        return true;
    }

    bool Write(uint8_t address, uint8_t reg, uint8_t value)
    {
        Wire.beginTransmission(address);
        bool complete = Wire.write(reg) == 1 && Wire.write(value) == 1;
        return Wire.endTransmission() == 0 && complete;
    }

    void DelayMs(uint32_t ms) { delay(ms); }
};
