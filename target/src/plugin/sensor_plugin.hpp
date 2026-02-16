/**
  ******************************************************************************
  * @file    sensor_plugin.hpp
  * @author  UrbanIzzy
  * @date    Feb 9, 2026
  * @brief   Base interface for all sensor plugins
  ******************************************************************************
*/
#pragma once

#include <string>
#include <memory>
#include <functional>
#include <chrono>
#include <map>
#include <thread>
#include <atomic>

namespace robotics {
namespace sensors {

using SensorDataCallBack = std::function<void(const void* data, size_t size)>;

enum class SensorStatus {
    UNINITIALIZED,
    INITIALIZED,
    RUNNING,
    STOPPED,
    ERROR
};

enum class SensorCapability : uint32_t {
    NONE = 0,
    ACCELEROMETER = 1<< 0,
    GYROSCOPE = 1<< 1,
    MAGNETOMETER = 1<< 2,
    BAROMETRE = 1<< 3,
    TEMPERATURE = 1<< 4,
    HUMIDITY = 1<< 5,
    GPS = 1<< 6,
    LIDAR = 1<< 7,
    CAMERA = 1<< 8,
    ULTRASONIC = 1<< 9
};

inline SensorCapability operator|(SensorCapability a, SensorCapability b) {
    return static_cast<SensorCapability>(
            static_cast<uint32_t>(a) | static_cast<uint32_t>(b)
    );
}

inline bool hasCapability(SensorCapability caps, SensorCapability check) {
    return (static_cast<uint32_t>(caps) & static_cast<uint32_t>(check)) != 0;
}

struct SensorMetadata {
    std::string name;
    std::string description;
    std::string version;
    std::string author;
    SensorCapability capabilities;
};

class SensorPlugin {
public:
    virtual ~SensorPlugin() = default;
    virtual bool initialize() = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual bool reset() = 0;

    virtual SensorMetadata getMetadata() const = 0;
    virtual SensorStatus getStatus() const = 0;
    virtual std::string getLastError() const = 0;
    
    virtual bool setConfig(const std::string& key, const std::string& value) {
        (void)key;
        (void)value;
        return false;       
    }

    virtual std::string getConfig(const std::string& key) const {
        (void)key;
        return "";
    }

    virtual bool selfTest() {
        return true; 
    }

    virtual std::map<std::string, std::string> getDiagnostics() const{
        return {};
    }

    bool isInitialized() const {
        auto status = getStatus();
        return status != SensorStatus::UNINITIALIZED;
    }

    bool isRunning() {
        auto status = getStatus();
        return status != SensorStatus::RUNNING;
    }

    bool hasError() {
        auto status = getStatus();
        return status != SensorStatus::ERROR;
    }

    std::string getName() const {
        return getMetadata().name;
    }

    std::string getDesription() const {
        return getMetadata().description;
    }
    
    std::string getVersion() const {
        return getMetadata().version;
    }
};

using SensorPluginFactory = std::function<std::unique_ptr<SensorPlugin>(const std::string& config)>;

} // namespace sensors
} // namespace robotics


