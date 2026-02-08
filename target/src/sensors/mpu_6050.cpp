/**
 * @file mpu_6050.cpp
 * @brief 
 * 
 */
/* Includes ------------------------------------------------------------------*/
 #include "mpu_6050.hpp"
 #include <stdint.h>
 #include <thread>
 #include <chrono>

namespace robotics {
namespace sensors {

MPU6050::MPU6050(int bus_num, uint8_t addr, const MPU6050Config &config)
: I2CDev(bus_num, addr)
, _config(config)
, _accel_scale(0)
, _gyro_scale(0)
{
    _accel_bias[0] = _accel_bias[1] =_accel_bias[2] = 0.0f;
    _gyro_bias[0] = _gyro_bias[1] =_gyro_bias[2] = 0.0f;
}

bool MPU6050::init(){
    if(!initDevice()){
        return false;
    }

    if(!verifyDeviceID(WHO_AM_I_REG, I_AM_MPU6050)){
        return false;
    }
    
    if(!reset()){
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if(!configSensor()){
        return false;
    }

    calcScaleFactors();
    return true;
}

bool MPU6050::reset(){
    if(!writeReg(PWR_MGMT_1_REG, 0x80)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

bool MPU6050::configSensor(){
    if(!writeReg(PWR_MGMT_1_REG, 0x00)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    uint8_t cfg;
    cfg = (static_cast<uint8_t>(_config.gyro_range) << 3);
    if(!writeReg(GYRO_CONFIG_REG, cfg)){
        return false;
    }
    
    cfg = (static_cast<uint8_t>(_config.accel_range) << 3);
    if(!writeReg(ACCEL_CONFIG_REG, cfg)){
        return false;
    }
    
    cfg = static_cast<uint8_t>(_config.dlp_filter);
    if(!writeReg(CONFIG_REG, cfg)){
        return false;
    }

    uint8_t div = (8000 / _config.sample_rate_hz) -1;
    if(!writeReg(SMPLRT_DIV_REG, div)){
        return false;
    }
    return true;
}

void MPU6050::calcScaleFactors(){
    float accel_lsb;
    switch(_config.accel_range){
        case MPU6050Config::AccelRange::RANGE_2G:  accel_lsb = 16384.0f; break;
        case MPU6050Config::AccelRange::RANGE_4G:  accel_lsb = 8192.0f;  break;
        case MPU6050Config::AccelRange::RANGE_8G:  accel_lsb = 4092.0f;  break;
        case MPU6050Config::AccelRange::RANGE_16G: accel_lsb = 2048.0f;  break;
        default: accel_lsb = 4092.0f;
    }
    _accel_scale = 9.80665f / accel_lsb;

    float gyro_lsb;
    switch(_config.gyro_range){
        case MPU6050Config::GyroRange::RANGE_250DPS:  gyro_lsb = 131.0f; break;
        case MPU6050Config::GyroRange::RANGE_500DPS:  gyro_lsb = 65.5f;  break;
        case MPU6050Config::GyroRange::RANGE_1000DPS: gyro_lsb = 32.8f;  break;
        case MPU6050Config::GyroRange::RANGE_2000DPS: gyro_lsb = 16.4f;  break;
        default:gyro_lsb = 32.8f;
    }
    gyro_lsb = (3.14159267f /180.0F) / gyro_lsb;
}

bool MPU6050::readData(MPU6050Data &data){
    uint8_t buffer[14];

    if(!readBrust(ACCEL_XOUT_H_REG, buffer, 14)){
        return false;
    }

    data.accel_raw[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    data.accel_raw[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    data.accel_raw[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

    data.temperature_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    data.gyro_raw[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
    data.gyro_raw[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
    data.gyro_raw[2] = (int16_t)((buffer[12] << 8) | buffer[13]);

    data.accel_x = data.accel_raw[0] * _accel_scale - _accel_bias[0];
    data.accel_y = data.accel_raw[1] * _accel_scale - _accel_bias[1];
    data.accel_z = data.accel_raw[2] * _accel_scale - _accel_bias[2];

    data.temperature = (data.temperature_raw / 340.f) +36.53f;
    
    data.gyro_x = data.gyro_raw[0] * _gyro_scale - _gyro_bias[0];
    data.gyro_y = data.gyro_raw[1] * _gyro_scale - _gyro_bias[1];
    data.gyro_z = data.gyro_raw[2] * _gyro_scale - _gyro_bias[2];

    return true;
}

bool MPU6050::readAccel(float& x, float& y, float& z){
    MPU6050Data data;
    
    if(!readData(data)){
        return false;
    }
    x = data.accel_x;
    y = data.accel_y;
    z = data.accel_z;
    return true;
}

bool MPU6050::readGyro(float& x, float& y, float& z){
    MPU6050Data data;
    
    if(!readData(data)){
        return false;
    }
    x = data.gyro_x;
    y = data.gyro_y;
    z = data.gyro_z;
    return true;
}

bool MPU6050::readTemp(float& temp){
    MPU6050Data data;
    
    if(!readData(data)){
        return false;
    }
    temp = data.temperature;
    return true;
}

bool MPU6050::calibrateGyro(int sample_num){
    if(sample_num <= 0)
        return false;

    float sum_x = 0, sum_y = 0,sum_z = 0;
    for(int i = 0; i < sample_num; i++){
        MPU6050Data data;
        if(!readData(data)){
            return false;
        }

        sum_x += data.gyro_raw[0] * _gyro_scale;
        sum_y += data.gyro_raw[1] * _gyro_scale;
        sum_z += data.gyro_raw[2] * _gyro_scale;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    _gyro_bias[0] = sum_x / sample_num;
    _gyro_bias[1] = sum_y / sample_num;
    _gyro_bias[2] = sum_z / sample_num;

    return true;
}

bool MPU6050::calibrateAccel(int sample_num){
    if(sample_num <= 0)
        return false;

    float sum_x = 0, sum_y = 0,sum_z = 0;
    for(int i = 0; i < sample_num; i++){
        MPU6050Data data;
        if(!readData(data)){
            return false;
        }

        sum_x += data.accel_raw[0] * _accel_scale;
        sum_y += data.accel_raw[1] * _accel_scale;
        sum_z += data.accel_raw[2] * _accel_scale;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    _accel_bias[0] = sum_x / sample_num;
    _accel_bias[1] = sum_y / sample_num;
    _accel_bias[2] = (sum_z / sample_num) - 9.80665f;

    return true;
}

bool MPU6050::selfTest(){
    uint8_t old_accel_cfg, old_gyro_cfg;
    if(!readReg(GYRO_CONFIG_REG, old_gyro_cfg) ||
       !readReg(ACCEL_CONFIG_REG, old_accel_cfg)){
        return false;
    }

    if(!writeReg(GYRO_CONFIG_REG, 0xF0) || 
       !writeReg(ACCEL_CONFIG_REG, 0xF0)){
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    MPU6050Data data;
    if(!readData(data)){
        writeReg(GYRO_CONFIG_REG, old_gyro_cfg);
        writeReg(ACCEL_CONFIG_REG, old_accel_cfg);
        return false;
    }

    bool gyro_ok = (std::abs(data.gyro_raw[0]) >= 100 &&
                    std::abs(data.gyro_raw[1]) >= 100 &&
                    std::abs(data.gyro_raw[2]) >= 100);

    bool accel_ok = (std::abs(data.accel_raw[0]) >= 100 &&
                    std::abs(data.accel_raw[1]) >= 100 &&
                    std::abs(data.accel_raw[2]) >= 100);
                    
    writeReg(GYRO_CONFIG_REG, old_gyro_cfg);
    writeReg(ACCEL_CONFIG_REG, old_accel_cfg);

    if(!gyro_ok || !accel_ok)
        return false;
    return true;
}

}
}