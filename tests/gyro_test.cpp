// Host tests of gyro identity selection, signed rates and transfer failures.
// Fake registers are test fixtures, not a physical sensor simulation.
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include "../firmware/Micromouse/GyroDriver.h"
#include "../firmware/Micromouse/Config.h"

int checks = 0;
void Check(bool condition, const char* message)
{
    checks++;
    if( !condition ) throw std::runtime_error(message);
}

struct FakeRegisters
{
    uint8_t registers[4][256] = {};
    uint8_t address = 0x68;
    uint8_t bank = 0;
    bool icm = false;
    bool failReads = false, failWrites = false, shortPayload = false, corruptReadback = false;
    int writes = 0, rateReads = 0;

    explicit FakeRegisters(uint8_t id, uint8_t addr = 0x68) : address(addr), icm(id == 0xEA)
    { registers[0][icm ? 0x00 : 0x75] = id; }

    bool Read(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t count)
    {
        if( addr != address || failReads ) return false;
        if( count == 2 )
        {
            rateReads++;
            if( shortPayload ) return false;
        }
        for( uint8_t i = 0; i < count; i++ ) data[i] = registers[bank][reg + i];
        if( corruptReadback && reg == (icm ? 0x06 : 0x6B) ) data[0] ^= 1;
        return true;
    }

    bool Write(uint8_t addr, uint8_t reg, uint8_t value)
    {
        writes++;
        if( failWrites || addr != address ) return false;
        if( icm && reg == 0x7F ) bank = (value >> 4) & 3;
        else registers[bank][reg] = value;
        return true;
    }
    void DelayMs(uint32_t) {}

    void Sample(int16_t raw, bool ready = true)
    {
        registers[0][icm ? 0x1A : 0x3A] = ready ? 1 : 0;
        uint8_t reg = icm ? 0x37 : 0x47;
        uint16_t value = static_cast<uint16_t>(raw);
        registers[0][reg] = value >> 8;
        registers[0][reg + 1] = value & 0xFF;
    }
};

void ModelTests(uint8_t id, uint8_t address, const char* name)
{
    FakeRegisters bus(id, address);
    GyroDriver<FakeRegisters> gyro(bus);
    Check(gyro.Begin(), "stock gyro initialization failed");
    Check(gyro.Address() == address && std::strcmp(gyro.Name(), name) == 0, "wrong gyro identity/address");
    Check(bus.bank == 0, "ICM did not restore bank zero");
    float rate = 999.0f;
    bus.Sample(6550, false);
    Check(gyro.ReadRate(rate) == GyroSample::Waiting && rate == 999 && bus.rateReads == 0,
          "stale gyro sample accepted");
    bus.Sample(6550);
    Check(gyro.ReadRate(rate) == GyroSample::Ready && std::fabs(rate - 100) < 0.001f,
          "positive rate scaling wrong");
    bus.Sample(-6550);
    Check(gyro.ReadRate(rate) == GyroSample::Ready && std::fabs(rate + 100) < 0.001f,
          "negative two's-complement rate wrong");
    bus.shortPayload = true;
    Check(gyro.ReadRate(rate) == GyroSample::Error, "short gyro packet accepted");
    bus.shortPayload = false;
    bus.failReads = true;
    Check(gyro.ReadRate(rate) == GyroSample::Error, "failed gyro transfer accepted");
    bus.failReads = false;
    bus.Sample(32767);
    Check(gyro.ReadRate(rate) == GyroSample::Error, "positive gyro clipping accepted");
    bus.Sample(-32768);
    Check(gyro.ReadRate(rate) == GyroSample::Error, "negative gyro clipping accepted");
}

int main()
{
    ModelTests(0x70, 0x68, "MPU-6500");
    ModelTests(0x71, 0x69, "MPU-9250");
    ModelTests(0xEA, 0x68, "ICM-20948");
    ModelTests(0xEA, 0x69, "ICM-20948");
    FakeRegisters unknown(0x68);
    GyroDriver<FakeRegisters> unsupported(unknown);
    Check(!unsupported.Begin() && unknown.writes == 0, "unknown gyro was configured");
    float rate = 0;
    Check(unsupported.ReadRate(rate) == GyroSample::Error, "uninitialized gyro returned a rate");
    FakeRegisters broken(0x70);
    broken.failWrites = true;
    GyroDriver<FakeRegisters> failWrite(broken);
    Check(!failWrite.Begin(), "failed configuration write ignored");
    FakeRegisters mismatch(0xEA);
    mismatch.corruptReadback = true;
    GyroDriver<FakeRegisters> readback(mismatch);
    Check(!readback.Begin(), "configuration readback mismatch ignored");
    const int pins[] = {Config::SdaPin, Config::SclPin, Config::LeftXshutPin,
                       Config::FrontXshutPin, Config::RightXshutPin, Config::FloorPin,
                       Config::StartPin, Config::LeftEnablePin, Config::LeftIn1Pin,
                       Config::LeftIn2Pin, Config::RightEnablePin, Config::RightIn1Pin, Config::RightIn2Pin};
    bool validPins = true;
    for( unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++ )
    {
        if( pins[i] == 16 || pins[i] == 17 || (pins[i] >= 6 && pins[i] <= 11) ) validPins = false;
        for( unsigned j = 0; j < i; j++ ) if( pins[i] == pins[j] ) validPins = false;
    }
    Check(validPins, "duplicate pin or WROVER flash/PSRAM pin selected");
    Check(Config::MinimumRangeMm < Config::SideEmergencyMm &&
          Config::SideEmergencyMm < Config::TargetWallMm, "distance thresholds inconsistent");
    std::cout << checks << " gyro/config assertions passed\n";
}
