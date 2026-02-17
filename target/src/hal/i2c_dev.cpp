/**
 * @file i2c_device.cpp
 * @brief Abstract base class for I2C devices
 * */

/* Includes ------------------------------------------------------------------*/
#include "i2c_dev.hpp"
#include <unistd.h>
#include <cstring>

namespace robotics {
namespace hal{

I2CDev::I2CDev(int bus_num, uint8_t addr) : I2CBus(addr)
    , _addr(addr) 
    , _bus_num(bus_num){}

bool I2CDev::initDevice(const I2CBusConfig& config){
    if(!open(_bus_num, config)){
        return false;
    }
    if(!ping()){
        close();
        return false;
    }
    return true;
}

bool I2CDev::_verifyAddr(){
    return setSlaveAddr(_addr);
}

bool I2CDev::readReg(uint8_t reg, uint8_t &data){
    if(!_verifyAddr()){
        return false;
    } 
    return writeReadData(&reg, 1, &data, 1);
}

bool I2CDev::writeReg(uint8_t reg, uint8_t data){
    if(!_verifyAddr()){
        return false;
    } 
    uint8_t buf[2] = {reg, data};
    if(writeData(buf, 2) != 2){
        return false;
    }
    return true;
}

bool I2CDev::modifyReg(uint8_t reg, uint8_t make, uint8_t data){
    uint8_t curr;
    if(!readReg(reg, curr)){
        return false;
    }
    curr &= ~make;
    curr |= (data & make);
    return writeReg(reg, curr);
}

bool I2CDev::readBrust(uint8_t s_reg, uint8_t *data, uint8_t len){
    if(!_verifyAddr()){
        return false;
    } 
    return writeReadData(&s_reg, 1, data, len);
}

bool I2CDev::writeBrust(uint8_t s_reg, const uint8_t *data, uint8_t len){
    if(!_verifyAddr()){
        return false;
    } 
    std::vector<uint8_t> buf(len + 1);
    buf[0] = s_reg;
    std::memcpy(&buf[1], data, len);
    if(writeData(buf.data(), len + 1) != static_cast<int>(len + 1)){
        return false;
    }
    return true;
}

bool I2CDev::readWord(uint8_t reg, uint16_t &data){
    uint8_t buf[2];
    if(!readBrust(reg, buf, 2)){
        return false;
    }
    data = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return true;
}

bool I2CDev::writeWord(uint8_t reg, uint16_t data){
    uint8_t buf[2]= {
        static_cast<uint8_t>(data >> 8), 
        static_cast<uint8_t>(data & 0xFF)};
    return writeBrust(reg, buf, 2);
}

bool I2CDev::readWordLe(uint8_t reg, uint16_t &data){
    uint8_t buf[2];
    if(!readBrust(reg, buf, 2)){
        return false;
    }
    data = (static_cast<uint16_t>(buf[1]) << 8) | buf[0];
    return true;
}

bool I2CDev::writeWordLe(uint8_t reg, uint16_t data){
    uint8_t buf[2]= {
        static_cast<uint8_t>(data & 0xFF),
        static_cast<uint8_t>(data >> 8)};
    return writeBrust(reg, buf, 2);
}

bool I2CDev::readInt16(uint8_t reg, int16_t &data){
    uint16_t udata;
    if(!readWord(reg, udata)){
        return false;
    }
    data = static_cast<int16_t>(udata);
    return true;
}

bool I2CDev::readInt16Le(uint8_t reg, int16_t &data){
    uint16_t udata;
    if(!readWordLe(reg, udata)){
        return false;
    }
    data = static_cast<int16_t>(udata);
    return true;
}

bool I2CDev::ping(){
    if (!_verifyAddr()){
        return false;
    }

    uint8_t dummy;
    if(readData(&dummy, 1)){
        return true;
    }
    return errno != ENXIO;
}

bool I2CDev::verifyDeviceID(uint8_t reg, uint8_t expected_id){
    uint8_t dev_id;

    if(!readReg(reg, dev_id)){
        return false;
    }
    return dev_id == expected_id;
}

}   // namespace hal
} // namespace robotics 