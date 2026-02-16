/**
  ******************************************************************************
  * @file    mpu_6050_plugin.hpp
  * @author  UrbanIzzy
  * @date    Feb 11, 2026
  * @brief   MPU6050 sensor plugin implementation
  ******************************************************************************
*/
#pragma once

#include "sensor_plugin.hpp"
#include "../sensors/mpu_6050.hpp"

#include <memory>
#include <functional>

namespace robotics {
namespace sensors {
    
using MPU6050CallBack = std::function<void(const MPU6050Data&)>;

class MPU6050Plugin : public SensorPlugin {
    public:
        static std::unique_ptr<MPU6050Plugin> create(const std::string& config);
        MPU6050Plugin (int bus_num, uint8_t addr = 0x68, const MPU6050Config& config = {});
        ~MPU6050Plugin() override;

        bool initialize() override;
        bool start() override;
        void stop() override;
        bool reset() override;

        SensorMetadata getMetadata() const override;
        SensorStatus getStatus() const override;
        std::string getLastError() const override;

        bool setConfig(const std::string& key, const std::string& value) override;
        std::string getConfig(const std::string& key) const override;

        bool selfTest() override;
        std::map<std::string, std::string> getDiagnostics() const override;


    /*------------------------------------------------------------------*/

        bool readSensorData(MPU6050Data& data);
        void setDataCallback(MPU6050CallBack& callback);
        bool calibrateGyro(int sample_num = 1000);
        bool calibrateAccel(int sample_num = 1000);

        MPU6050& getDevice() { return *_device; }
        const MPU6050& getDevice() const { return *_device; }

    private:
        void threadFunction();             

        std::unique_ptr<MPU6050> _device;
        SensorStatus _status;
        std::string _last_error;

        MPU6050CallBack _dataCallback;
        std::unique_ptr<std::thread> _thread;
        std::atomic<bool> _thread_is_running;
};


}
}