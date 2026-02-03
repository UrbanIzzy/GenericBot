/**
  ******************************************************************************
  * @file    i2c_dev.c
  * @author  UrbanIzzy
  * @date    Feb 01, 2026
  * @brief   This file contain class headers of I2C device management
  ******************************************************************************
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "i2c_bus.hpp"
#include <cstdint>
#include <vector>
#include <string>

namespace robotics {
namespace hal{ 
/* Classes -------------------------------------------------------------------*/
class I2CDev : public I2CBus{
    public:
        virtual ~I2CDev() = default;
        
        virtual bool init() = 0;
        uint8_t getAddr() const { return _addr; }
        
        bool readReg(uint8_t reg, uint8_t &data);
        bool writeReg(uint8_t reg, uint8_t data);
        bool modifyReg(uint8_t reg, uint8_t make, uint8_t data);

        bool readBrust(uint8_t s_reg, uint8_t *data, uint8_t len);
        bool writeBrust(uint8_t s_reg, const uint8_t *data, uint8_t len);

        bool readWord(uint8_t reg, uint16_t &data);
        bool writeWord(uint8_t reg, uint16_t data);

        bool readWordLe(uint8_t reg, uint16_t &data);
        bool writeWordLe(uint8_t reg, uint16_t data);

        bool readInt16(uint8_t reg, int16_t &data);
        bool readInt16Le(uint8_t reg, int16_t data);

        bool ping();
        bool verifyDeviceID(uint8_t reg, uint8_t expected_id);

    protected:
        uint8_t _addr;
        uint8_t _bus_num;
        I2CDev(int bus_num, uint8_t addr);
        bool initDevice(const I2CBusConfig &config = {});

    private:
        bool _verifyAddr();
};

} // namespace hal
} // namespace robotics