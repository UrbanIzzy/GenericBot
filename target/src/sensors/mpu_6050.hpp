/**
  ******************************************************************************
  * @file    mpu_6500.c
  * @author  UrbanIzzy
  * @date    Feb 2, 2026
  * @brief   This file contain class headers of IMU6500 driver
  ******************************************************************************
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "../hal/i2c_dev.hpp"
#include <array>

namespace robotics {
namespace sensors {

/* Definitions --------------------------------------------------------------*/
struct MPU6050Config {
  enum class AccelRange {
    RANGE_2G = 0,
    RANGE_4G = 1,
    RANGE_8G = 2,
    RANGE_16G = 3
  } accel_range = AccelRange::RANGE_2G;

  enum class GyroRange {
    RANGE_250DPS = 0,
    RANGE_500DPS = 1,
    RANGE_1000DPS = 2,
    RANGE_2000DPS = 3
  } gyro_range = GyroRange::RANGE_250DPS;

  enum class DLPF {
    DLPF_260HZ = 0,
    DLPF_184HZ = 1,
    DLPF_94HZ  = 2,
    DLPF_44HZ  = 3,
    DLPF_21HZ  = 4,
    DLPF_10HZ  = 5,
    DLPF_5HZ   = 6
  } dlp_filter = DLPF::DLPF_44HZ;
  
  uint16_t sample_rate_hz = 1000;  // Sample rate in Hz
};

struct MPU6050Data{
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  // float mag_x, mag_y, mag_z;
  float temperature;

  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  // int16_t mag_raw[3];
  int16_t temperature_raw;
  
  uint64_t timestamp;
};

class MPU6050 : public hal::I2CDev {
public:
    MPU6050(int bus_num, uint8_t addr = 0x68, const MPU6050Config &config = {});

    bool init() override;
    bool readData(MPU6050Data &data);
    
    bool readAccel(float& x, float& y, float& z);
    bool readGyro(float& x, float& y, float& z);
    // bool readMag(float& x, float& y, float& z);
    bool readTemp(float& temp);

    bool calibrateGyro(int sample_num = 1000);
    bool calibrateAccel(int sample_num = 1000);
    
    bool configSensor();

    void getGyroBais(float& x, float& y, float& z) const {
      x = _gyro_bias[0];
      y = _gyro_bias[1];
      z = _gyro_bias[2];
    }
    
    void setGyroBais(float x, float y, float z){ 
      _gyro_bias[0] = x;
      _gyro_bias[1] = y;
      _gyro_bias[2] = z;
    }

    bool setConfig(const MPU6050Config& config){
      _config = config;
      if(!configSensor())
        return false;
      calcScaleFactors();
      return true;  
    }

    bool reset();
    bool selfTest();

    const MPU6050Config& getConfig() const { return _config; }
    MPU6050Config& getConfig() { return _config; }

  private:
    MPU6050Config _config;

    float _accel_scale;
    float _gyro_scale;

    float _accel_bias[3] = {0, 0, 0};
    float _gyro_bias[3] = {0, 0, 0};

    static constexpr uint8_t WHO_AM_I_REG = 0x75;
    static constexpr uint8_t PWR_MGMT_1_REG = 0x68;
    static constexpr uint8_t PWR_MGMT_2_REG = 0x6C;
    static constexpr uint8_t SMPLRT_DIV_REG = 0x19;
    static constexpr uint8_t CONFIG_REG =  0x1A;
    static constexpr uint8_t GYRO_CONFIG_REG = 0x18;
    static constexpr uint8_t ACCEL_CONFIG_REG = 0x1C;
    static constexpr uint8_t ACCEL_XOUT_H_REG = 0x38;
    static constexpr uint8_t TEMP_OUT_H_REG = 0x41;
    static constexpr uint8_t GYRO_XOUT_REG = 0x43;

    static constexpr uint8_t I_AM_MPU6050 = 0x68;

    void calcScaleFactors();
  };

} // namespace sensor
} // namespace robotics