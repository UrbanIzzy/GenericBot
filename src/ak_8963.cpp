/**
 * @file ak_8963.cpp
 * @brief AK8963 3-axis magnetometer sensor driver implemetation
 */

/* Includes ------------------------------------------------------------------*/
#include "ak_8963.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>

namespace robotics {
namespace sensors{

AK8963::AK8963(int bus_num, uint8_t addr, const AK8963Config &config)
    : I2CDev(bus_num, addr)
    , _config(config)
    , _mag_scale(0){
        _asa[0] = _asa[1] = _asa[2] = 1.0f;
        _hard_iron_bias[0] = _hard_iron_bias[1] = _hard_iron_bias[2] = 0.0f;
}

bool AK8963::init(){
    if(!initDevice()){
        return false;
    }

    if(!verifyDeviceID(WAI, IAM)){
        return false;
    }

    if(!readSensitivityAdjustment()){
        return false;
    }

    if(!configSensor()){
        return false;
    }

    calcScaleFactor();
    return true;
}

bool AK8963::readSensitivityAdjustment(){
    if(!writeReg(CNTL1, 0x0F)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint8_t asa[3];
    if(!readBrust(ASAX, asa, 3)){
        return false;
    }

    _asa[0] = ((float)asa[0] - 128.0f) / 256.0f + 1.0f;
    _asa[1] = ((float)asa[1] - 128.0f) / 256.0f + 1.0f;
    _asa[2] = ((float)asa[2] - 128.0f) / 256.0f + 1.0f;

    if(!writeReg(CNTL1, 0x00)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
}

void AK8963::calcScaleFactor(){
    if(_config.resolution == AK8963Config::Resolution::BIT_14){
        _mag_scale = 0.6f;
    }
    else{
        _mag_scale = 0.15f;
    }
}

bool AK8963::configSensor(){
    uint8_t ctrl = static_cast<uint8_t>(_config.mode);
    if(_config.resolution == AK8963Config::Resolution::BIT_16){
        ctrl |= 0x10;
    }

    if(!writeReg(CNTL1, ctrl)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
}

bool AK8963::reset() {
    if(!writeReg(CNTL2, 0x01)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
}

bool AK8963::readData(AK8963Data& data){
    uint8_t status;
    if(!readReg(ST1, status)){
        return false;
    }

    data.is_ready = (status & 0x01);
    if(!data.is_ready){
        return true;
    }

    uint8_t buffer[7];
    if(!readBrust(HXL, buffer, 7)){
        return false;
    }

    if(buffer[6] == 0x08){
        return false;
    }

    data.mag_raw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    data.mag_raw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    data.mag_raw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);

    data.asa[0] = _asa[0];
    data.asa[1] = _asa[1];
    data.asa[2] = _asa[2];
    return true;
}

bool AK8963::calibHardIron(int sample_num){
    if(sample_num <= 0){
        return false;
    }

    float min_x = 1e6f, max_x = -1e6f;
    float min_y = 1e6f, max_y = -1e6f;
    float min_z = 1e6f, max_z = -1e6f;

    for(int i = 0; i < sample_num; i++){
        AK8963Data data;
        if(!readData(data) || !data.is_ready){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        float x = data.mag_raw[0] * _mag_scale * _asa[0];
        float y = data.mag_raw[1] * _mag_scale * _asa[1];
        float z = data.mag_raw[2] * _mag_scale * _asa[2];

        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        min_z = std::min(min_z, z);
        
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
        max_z = std::max(max_z, z);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    _hard_iron_bias[0] = (min_x + max_x) / 2.0f;
    _hard_iron_bias[1] = (min_y + max_y) / 2.0f;
    _hard_iron_bias[2] = (min_z + max_z) / 2.0f;
    return true;
}

bool AK8963::selfTest(){
    uint8_t old_mode;
    if(!readReg(CNTL1, old_mode)){
        return false;
    }

    if(!writeReg(ASTC, 0x40)){
        return false;
    }

    uint8_t ctrl = 0x08;
    if(_config.resolution == AK8963Config::Resolution::BIT_16){
        ctrl |= 0x10;
    }

    if(!writeReg(CNTL1, ctrl)){
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    AK8963Data data;
    bool ret = readData(data);

    writeReg(ASTC, 0x00);
    writeReg(CNTL1, old_mode);

    if(!ret){
        return false;
    }

    bool test_ok = (std::abs(data.mag_raw[0]) > 50 &&
                    std::abs(data.mag_raw[1]) > 50 &&
                    std::abs(data.mag_raw[2]) > 50);
    
    if(!test_ok){
        return false;
    }
    return true;
}

}
}