/**
 * @file mpu6050_pulgin.cpp
 * @brief MPU6050 sensor plugin implementation
 */

#include "mpu6050_plugin.hpp"
#include "plugin_registry.hpp"
#include <sstream>

namespace robotics {
namespace sensors {

REGISTER_SENSOR_PULGIN(
    MPU6050Plugin,
    "mpu6050",
    "MPU6050 6-axis IMU (3-axis acclerometer + 3-axis gyrometer)"
);

MPU6050Plugin::MPU6050Plugin(int bus_num, uint8_t addr, const MPU6050Config& config)
    : _device(std::make_unique<MPU6050> (bus_num, addr, config))
    , _status(SensorStatus::UNINITIALIZED)
    , _thread_is_running(false) {

}

MPU6050Plugin::~MPU6050Plugin(){
    stop();
}

std::unique_ptr<MPU6050Plugin> MPU6050Plugin::create(const std::string& config){
    int bus_num = 2;
    uint8_t addr = 0x68;

    if(!config.empty()) {
        std::istringstream iss(config);
        char colon;

        if(iss >> bus_num >> colon >> std::hex >> (int&)addr) {

        }
    }
    return std::make_unique<MPU6050Plugin>(bus_num, addr);
}

bool MPU6050Plugin::initialize() {
    if(_status != SensorStatus::UNINITIALIZED) {
        _last_error = "Already initialized";
        return false;
    }

    if(!_device->init()) {
        _last_error = _device->getLastError();
        _status = SensorStatus::ERROR;
        return false;
    }

    _status = SensorStatus::INITIALIZED;
    return true;
}

bool MPU6050Plugin::start() {
    if(_status != SensorStatus::INITIALIZED && _status != SensorStatus::STOPPED) {
        _last_error = "Must be initialized before starting";
        return false;
    }

    if(_dataCallback) {
        _thread_is_running = true;
        _thread = std::make_unique<std::thread>(&MPU6050Plugin::threadFunction, this);
    }
    _status = SensorStatus::RUNNING;
    return true;
}

void MPU6050Plugin::stop() {
    if(_thread_is_running) {
        _thread_is_running = false;
        if(_thread && _thread->joinable()) {
            _thread->join();
        }
        _thread.reset();
    }

    if(_status == SensorStatus::RUNNING) {
        _status = SensorStatus::STOPPED;
    }
}

bool MPU6050Plugin::reset() {
    stop();

    if(!_device->reset()) {
        _last_error = _device->getLastError();
        _status = SensorStatus::ERROR;
        return false;
    }

    _status = SensorStatus::INITIALIZED;
    return true;
}

SensorMetadata MPU6050Plugin::getMetadata() const {
    return {
        .name = "MPU6050",
        .description = "InvenSense MPU6050 6-axis IMU",
        .version = "1.0.0",
        .author = "UrbanIzzy",
        .capabilities = SensorCapability::ACCELEROMETER |
                       SensorCapability::GYROSCOPE |
                       SensorCapability::TEMPERATURE
    };
}

SensorStatus MPU6050Plugin::getStatus() const {
    return _status;
}

std::string MPU6050Plugin::getLastError() const {
    return _last_error;
}

bool MPU6050Plugin::setConfig(const std::string& key, const std::string& value) {
    if(_status == SensorStatus::RUNNING) {
        _last_error = "cannot chage configuration while running";
        return false;
    }
    
    MPU6050Config config = _device->getConfig();
    try{
        if(key == "accel_range"){
            int range = std::stoi(value);
            switch(range) {
                case 2:
                    config.accel_range = MPU6050Config::AccelRange::RANGE_2G; 
                    break;
                case 4: 
                    config.accel_range = MPU6050Config::AccelRange::RANGE_4G;
                    break;
                case 8: 
                    config.accel_range = MPU6050Config::AccelRange::RANGE_8G;
                    break;
                case 16:
                    config.accel_range = MPU6050Config::AccelRange::RANGE_16G;
                    break;
                default:
                    _last_error = "invalid accel range value (allowed: 2, 4, 8, 16)";
                    return false;
            }
        }else if(key == "gyro_range"){
            int range = std::stoi(value);
            switch(range) {
                case 250:
                    config.gyro_range = MPU6050Config::GyroRange::RANGE_250DPS;
                    break;
                case 500: 
                    config.gyro_range = MPU6050Config::GyroRange::RANGE_500DPS;
                    break;
                case 1000: 
                    config.gyro_range = MPU6050Config::GyroRange::RANGE_1000DPS;
                    break;
                case 2000:
                    config.gyro_range = MPU6050Config::GyroRange::RANGE_2000DPS;
                    break;
                default:
                    _last_error = "invalid gyro range value (allowed: 250, 500, 1000, 2000)";
                    return false;
            }
        }else if(key == "sample_rate"){
            int rate = std::stoi(value);
            if(rate <= 0 || rate > 1000) {
                _last_error = "invalid sample rate value (allowed: 1-1000 Hz)";
                return false;
            }
            config.sample_rate_hz = rate;
        }else if(key == "dlpf"){
            int dlpf = std::stoi(value);
            if(dlpf < 0 || dlpf > 6) {
                _last_error = "invalid dlpf value (allowed: 0-6)";
                return false;
            }
            config.dlp_filter = static_cast<MPU6050Config::DLPF>(dlpf);
        } else {
            _last_error = "unknown configuration key";
            return false;
        }

        if(!_device->setConfig(config)) {
            _last_error = "Failed to apply dlpf configuration: ";
            return false;
        }
        return true;
    }catch(const std::exception& e) {
        _last_error = "Failed to parse configuration invalid value format: " + std::string(e.what());
        return false;
    }


    if(key == "accel_range"){
        // \tdb: add parsing and set accel erage
        return true;
    }else if(key == "gyro_range"){
        // \tdb: add parsing and set gyro rage
        return true;
    }
    return false;
}

std::string MPU6050Plugin::getConfig(const std::string& key) const {
    const auto& config = _device->getConfig();

    if(key == "accel_range"){
        return std::to_string(static_cast<int>(config.accel_range));
    }else if(key == "gyro_range"){
        return std::to_string(static_cast<int>(config.gyro_range));
    }else if(key == "sample_rate"){
        return std::to_string(static_cast<int>(config.sample_rate_hz));
    }
    return "";
}

bool MPU6050Plugin::selfTest() {
    return _device->selfTest();
}

std::map<std::string, std::string> MPU6050Plugin::getDiagnostics() const {
    std::map<std::string, std::string> diag;

    diag["status"] = [this]() {
        switch (_status) {
            case SensorStatus::UNINITIALIZED: return "uninitialized";
            case SensorStatus::INITIALIZED: return "initialized";
            case SensorStatus::RUNNING: return "running";
            case SensorStatus::STOPPED: return "stopped";
            case SensorStatus::ERROR: return "error";
            default: return "unknowen";
        }
    }();

    diag["address"] = std::to_string(_device->getAddr());
    diag["bus"] = std::to_string(_device->getBusNum());


    auto stat = _device->getState();
    diag["total_tarnsactions"] = std::to_string(stat.total_tarnsactions);
    diag["failed_transactions"] = std::to_string(stat.failed_transactions);
    diag["bus_recoveries"] = std::to_string(stat.bus_recoveries);

    return diag;
}

bool MPU6050Plugin::readSensorData(MPU6050Data& data){
    if(_status != SensorStatus::RUNNING && _status != SensorStatus::INITIALIZED) {
        _last_error = "sensor not ready";
        return false;
    }
    if(_device->readData(data)) {
        _last_error = _device->getLastError();
        return false;
    }
    return true;
}

void MPU6050Plugin::setDataCallback(MPU6050CallBack& callback) {
    _dataCallback = callback;
}

bool MPU6050Plugin::calibrateGyro(int sample_num) {
    return _device->calibrateGyro(sample_num);
}

bool MPU6050Plugin::calibrateAccel(int sample_num) {
    return _device->calibrateAccel(sample_num);       
}

void MPU6050Plugin::threadFunction() {
    while (_thread_is_running) {
        MPU6050Data data;
        
        if(_device->readData(data)){
            if(_dataCallback) {
                _dataCallback(data);
            }
        }
        const auto& config = _device->getConfig();
        std::this_thread::sleep_for(std::chrono::microseconds(1000000 / config.sample_rate_hz));
    }
}

} // namespace sensors
} // namespace robotics