// Z-axis only. Register maps: InvenSense MPU-6500/9250 and ICM-20948 DS-000189.
// No magnetometer/DMP/position estimation. RegisterBus checks complete transfers.
#pragma once
#include <stdint.h>

enum class GyroModel { Unknown, MPU6500, MPU9250, ICM20948 };
enum class GyroSample { Waiting, Ready, Error };

template<class RegisterBus>
class GyroDriver
{
public:
    explicit GyroDriver(RegisterBus& bus) : mBus(bus) {}

    bool Begin()
    {
        mModel = GyroModel::Unknown;
        for( uint8_t address = 0x68; address <= 0x69; address++ )
        {
            uint8_t id = 0;
            if( mBus.Read(address, 0x75, &id, 1) && (id == 0x70 || id == 0x71) )
                mModel = id == 0x70 ? GyroModel::MPU6500 : GyroModel::MPU9250;
            else if( mBus.Read(address, 0x00, &id, 1) && id == 0xEA )
                mModel = GyroModel::ICM20948;
            else continue;
            mAddress = address;
            bool ok = mModel == GyroModel::ICM20948 ? InitIcm() : InitMpu();
            if( !ok ) mModel = GyroModel::Unknown;
            return ok;
        }
        return false;
    }

    GyroSample ReadRate(float& rateDps)
    {
        if( mModel == GyroModel::Unknown ) return GyroSample::Error;
        uint8_t status = 0;
        bool icm = mModel == GyroModel::ICM20948;
        if( icm && !mBus.Write(mAddress, 0x7F, 0x00) ) return GyroSample::Error;
        if( !mBus.Read(mAddress, icm ? 0x1A : 0x3A, &status, 1) ) return GyroSample::Error;
        if( !(status & 0x01) ) return GyroSample::Waiting;
        uint8_t bytes[2];
        if( !mBus.Read(mAddress, icm ? 0x37 : 0x47, bytes, 2) ) return GyroSample::Error;
        // Explicit two's-complement decoding, portable to the desktop tests.
        int32_t raw = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
        if( raw >= 32768 ) raw -= 65536;
        if( raw <= -32500 || raw >= 32500 ) return GyroSample::Error; // Clipping loses yaw.
        rateDps = raw / 65.5f; // Both devices are configured to +/-500 degrees/s.
        return GyroSample::Ready;
    }

    uint8_t Address() const { return mAddress; }
    const char* Name() const
    {
        switch( mModel )
        {
            case GyroModel::MPU6500: return "MPU-6500";
            case GyroModel::MPU9250: return "MPU-9250";
            case GyroModel::ICM20948: return "ICM-20948";
            default: return "unknown";
        }
    }

private:
    RegisterBus& mBus;
    GyroModel mModel = GyroModel::Unknown;
    uint8_t mAddress = 0;

    bool SetChecked(uint8_t reg, uint8_t value)
    {
        uint8_t actual = 0;
        return mBus.Write(mAddress, reg, value) &&
               mBus.Read(mAddress, reg, &actual, 1) && actual == value;
    }

    bool InitMpu()
    {
        if( !mBus.Write(mAddress, 0x6B, 0x80) ) return false; // Reset.
        mBus.DelayMs(100);
        if( !SetChecked(0x6B, 0x01) || !SetChecked(0x6C, 0x00) ||
            !SetChecked(0x1A, 0x04) || // DLPF about 20 Hz.
            !SetChecked(0x1B, 0x08) || // +/-500 dps, DLPF enabled.
            !SetChecked(0x19, 0x04) || // 1 kHz / 5 = 200 Hz.
            !SetChecked(0x38, 0x01) ) return false; // Data-ready status enabled.
        mBus.DelayMs(100);
        return true;
    }

    bool InitIcm()
    {
        if( !mBus.Write(mAddress, 0x7F, 0x00) || !mBus.Write(mAddress, 0x06, 0x80) ) return false;
        mBus.DelayMs(100);
        if( !mBus.Write(mAddress, 0x7F, 0x00) ||
            !SetChecked(0x06, 0x01) || !SetChecked(0x07, 0x00) ||
            !SetChecked(0x05, 0x00) || // Continuous low-noise operation.
            !SetChecked(0x11, 0x01) || // RAW_DATA_0_RDY interrupt enabled.
            !mBus.Write(mAddress, 0x7F, 0x20) ||
            !SetChecked(0x00, 0x04) || // 1125 Hz / 5 = 225 Hz.
            !SetChecked(0x01, 0x23) || // +/-500 dps, DLPF 23.9 Hz enabled.
            !mBus.Write(mAddress, 0x7F, 0x00) ) return false;
        mBus.DelayMs(100);
        return true;
    }
};
