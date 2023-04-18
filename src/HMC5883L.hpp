#pragma once

#include <exception>
#include <thread>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <math.h>

#include "i2c.h"

#define MAX_GAIN 7
using namespace std;

class HMC5883L
{
    public:

    enum class Units
    {
        G = 1,
        mG = 1000,
        uG = 1000000
    };

    enum class Axes
    {
        X,
        Y,
        Z
    };

    class sensor_error: exception
    {
        public:
        sensor_error(const std::string& message): message{message}
        {}
        const char* what() const noexcept override
        {
            return message.c_str();
        }
        private:
        string message;
    };

    struct PlainData_t
    {
        double absValue;
        double degrees;
    };

    HMC5883L(const string& i2cdev)
    {
        uint8_t data_setup[2] = {(uint8_t)Registers::ModeReg, 0x00};
        struct i2c_msg msg_setup =
        {
                .addr = (unsigned short)_sensorBusAddres,
                .flags = 0,
                .len = 2,
                .buf = data_setup
        };

        i2c = i2c_new();

        if (i2c_open(i2c, i2cdev.c_str()) < 0) 
            throw new sensor_error("Can't open I2C dev.");

        if (i2c_transfer(i2c, &msg_setup, 1) < 0) 
            throw new sensor_error("I2C transfer error");
    }

    ~HMC5883L()
    {
        i2c_close(i2c);

        if(i2c != nullptr)
            i2c_free(i2c);
    }

    bool Measure()
    {
        uint8_t msg_datareg[1] = {(uint8_t)Registers::DataRegMsbX};
        BinaryData_t binaryData;
        struct i2c_msg msgs[2] = 
        {
            {
                .addr = (unsigned short)_sensorBusAddres,
                .flags = 0,
                .len = 1,
                .buf = msg_datareg
            },
            {
                .addr = (unsigned short)_sensorBusAddres,
                .flags = I2C_M_RD,
                .len = 6,
                .buf = (uint8_t*)&binaryData
            }
        };

        // Getting data from sensor.
        if (i2c_transfer(i2c, msgs, 2) < 0)
            throw new sensor_error("I2C transfer error");

        binaryData.x = SwapBytes(binaryData.x);
        binaryData.y = SwapBytes(binaryData.y);
        binaryData.z = SwapBytes(binaryData.z);

        if(binaryData.x.i16 == -4096 || binaryData.y.i16 == -4096 || binaryData.z.i16 == -4096) return false;

        _data.x = Gains[_currentGain].resolution * binaryData.x.i16;
        _data.y = Gains[_currentGain].resolution * binaryData.y.i16;
        _data.z = Gains[_currentGain].resolution * binaryData.z.i16;

        return true;
    }

    void Calibrate()
    {
        double maxValue = 0.0;

        // Setting minimum gain for measuring range maximization.
        SetGain(MAX_GAIN);

        // Getting maximum value
        for(int i = 0; i < 3; i++)
        {
            // Overload at minimum gain.
            if(!Measure()) return;

            maxValue = max(abs(_data.x), abs(_data.y));
            maxValue = max(abs(_data.z), maxValue);
        }

        // Setting optimal gain.
        uint8_t gain = GetOptimalGain(maxValue);
        SetGain(gain);
    }

    void SetGain(uint8_t gain)
    {
        if(gain > MAX_GAIN) gain == MAX_GAIN;

        uint8_t data_setup[2] = {(uint8_t)Registers::ConfigRegB, (uint8_t)(gain << 5)};
        struct i2c_msg msg_setup =
        {
                .addr = (unsigned short)_sensorBusAddres,
                .flags = 0,
                .len = 2,
                .buf = data_setup
        };

        if (i2c_transfer(i2c, &msg_setup, 1) < 0) 
            throw new sensor_error("I2C transfer error");

        _currentGain = gain;

        this_thread::sleep_for(chrono::milliseconds(200));
    }

    // double GetGausesX(){return _data.x;}
    // double GetGausesY(){return _data.y;}
    // double GetGausesZ(){return _data.z;}

    double GetMagnitude(Axes axis, Units units = Units::G)
    {
        switch (axis)
        {
            case Axes::X: return _data.x * (double)units;
            case Axes::Y: return _data.y * (double)units;
            case Axes::Z: return _data.z * (double)units;
            default:return 0.0;
        }
    }
    PlainData_t GetPlainData(Axes firstAxis, Axes secondAxis, Units units = Units::G)
    {
        double firstMag  = GetMagnitude(firstAxis,  units);
        double secondMag = GetMagnitude(secondAxis, units);
        PlainData_t result = 
        {
            .absValue = sqrt(pow(firstMag, 2) + pow(secondMag, 2)),
            .degrees = atan2(secondMag, firstMag) * 180 / M_PI
        };

        return result;
    }

    private:

    enum class Registers
    {
        ConfigRegA = 0,
        ConfigRegB,
        ModeReg,
        DataRegMsbX,
        DataRegLsbX,
        DataRegMsbZ,
        DataRegLsbZ,
        DataRegMsbY,
        DataRegLsbY,
        StatusReg,
        IdentificationRegA,
        IdentificationRegB,
        IdentificationRegC
    };

    union DimData_t
    {
        int16_t  i16;
        uint16_t ui16;
        uint8_t bytes[2];
    };

    struct ProcessedData_t
    {
        double x;
        double y;
        double z;
    };

    struct BinaryData_t
    {
        DimData_t x;
        DimData_t z;
        DimData_t y;
    };

    struct SensorGain_t
    {
        uint8_t regValue;
        double maxAbsValue;
        double resolution;
    };

    const SensorGain_t Gains[8] = 
    {
        {
            .regValue = 0,
            .maxAbsValue = 0.88,
            .resolution = 1.0/1370
        },
        {
            .regValue = 1,
            .maxAbsValue = 1.3,
            .resolution = 1.0/1090
        },
        {
            .regValue = 2,
            .maxAbsValue = 1.9,
            .resolution = 1.0/820
        },
        {
            .regValue = 3,
            .maxAbsValue = 2.5,
            .resolution = 1.0/660
        },
        {
            .regValue = 4,
            .maxAbsValue = 4.0,
            .resolution = 1.0/440
        },
        {
            .regValue = 5,
            .maxAbsValue = 4.7,
            .resolution = 1.0/390
        },
        {
            .regValue = 6,
            .maxAbsValue = 5.6,
            .resolution = 1.0/330
        },
        {
            .regValue = 7,
            .maxAbsValue = 8.1,
            .resolution = 1.0/230
        },
    };

    const uint8_t _sensorBusAddres = 0x1E;
    uint8_t _currentGain = 1;
    ProcessedData_t _data;
    i2c_t *i2c = nullptr;

    DimData_t SwapBytes(DimData_t data)
    {
        DimData_t out;

        out.bytes[1] = data.bytes[0];
        out.bytes[0] = data.bytes[1];

        return out;
    }

    uint8_t GetOptimalGain(double maxDimValue)
    {
        double minDiff = 10000.0;
        uint8_t optimalGain = 0;
        for(int i = 0; i <= MAX_GAIN; i++)
        {
            double diff = abs(Gains[i].maxAbsValue/2 - abs(maxDimValue));

            if(minDiff > diff)
            {
                optimalGain = (uint8_t)i;
                minDiff = diff;
            }
        }

        return optimalGain;
    }
};