/**
  ******************************************************************************
  * @file    ak8963_device.hpp
  * @author  UrbanIzzy
  * @date    Feb 8, 2026
  * @brief   AK8963 3-axis magnetometer
  ******************************************************************************
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "i2c_dev.hpp"
#include <array>

namespace robotics {
namespace sensors {

struct AK8963Config {
    enum class Mode {
        POWER_DOWN = 0x00,
        SINGLE = 0x01,
        CONTINOUS_8HZ = 0x02,
        CONTINOUS_100HZ = 0x06
    } mode = Mode::CONTINOUS_100HZ;

    enum class Resolution {
        BIT_14 = 0,
        BIT_16 = 1
    } resolution = Resolution::BIT_16;
};

struct AK8963Data {
    float mag_x, mag_y, mag_z;
    int16_t mag_raw[3];
    float asa[3];
    bool is_ready = false;
    uint64_t timestamp_us = 0;
};

class AK8963 : public hal::I2CDev {
    public:
        AK8963(
            int busNumber,
            uint8_t addr = 0x0C,
            const AK8963Config& config = {}
        );

        bool init() override;
        bool readData(AK8963Data& data);
        bool readMag(float& x, float& y, float& z);

        bool calibHardIron(int sampal_num = 1000);
        void getHardIronBais(float& x, float& y, float& z) const {
            x = _hard_iron_bias[0];
            y = _hard_iron_bias[1];
            z = _hard_iron_bias[2];
        }
        void setHardIronBais(float& x, float& y, float& z){
            _hard_iron_bias[0] = x;
            _hard_iron_bias[1] = y;
            _hard_iron_bias[2] = z;
        }

        bool reset();
        bool selfTest();
        const AK8963Config& getConfig() const { return _config;}

    private:
        AK8963Config _config;
        float _asa[3] = {1.0f, 1.0f, 1.0f};
        float _hard_iron_bias[3] = {0, 0, 0};
        float _mag_scale;

        static constexpr uint8_t IAM = 0x48;

        static constexpr uint8_t WAI = 0x00;
        static constexpr uint8_t INFO = 0x01;
        static constexpr uint8_t ST1 = 0x02;
        static constexpr uint8_t HXL = 0x03;
        static constexpr uint8_t HXH = 0x04;
        static constexpr uint8_t HYL = 0x05;
        static constexpr uint8_t HYH = 0x06;
        static constexpr uint8_t HZL = 0x07;
        static constexpr uint8_t HZH = 0x08;
        static constexpr uint8_t ST2 = 0x09;
        static constexpr uint8_t CNTL1 = 0x0A;
        static constexpr uint8_t CNTL2 = 0x0B;
        static constexpr uint8_t ASTC = 0x0C;
        static constexpr uint8_t ASAX = 0x10;
        static constexpr uint8_t ASAY = 0x11;
        static constexpr uint8_t ASAZ = 0x12;

        bool readSensitivityAdjustment();
        void calcScaleFactor();
        bool configSensor();
};

}
}
