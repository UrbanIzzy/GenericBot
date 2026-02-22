/**
  ******************************************************************************
  * @file    serial_rc_plugin.hpp
  * @author  UrbanIzzy
  * @date    Feb 19, 2026
  * @brief   RC receiver plugin with dedicated background thread
  ******************************************************************************
*/
#pragma once

#include "../sensors/rc_base.hpp"
#include "sensor_plugin.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>
#include <functional>

namespace robotics {
namespace sensors {

class SerialRCPlugin : public SensorPlugin {
    public:
        static std::unique_ptr<SerialRCPlugin> create(const std::string& config);
        explicit SerialRCPlugin(std::unique_ptr<RCBase> rc_receiver);

        ~SerialRCPlugin() override;
        SerialRCPlugin(const SerialRCPlugin&) = delete;        

        bool initialize() override;
        bool start() override;
        void stop() override;
        bool reset() override;

        SensorMetadata getMetadata() const override;
        SensorStatus getStatus() const override { return _status; }
        std::string getLastError() const override;

        bool setConfig(const std::string& key, const std::string& value) override;
        std::string getConfig(const std::string& key) const override;
        std::map<std::string, std::string> getDiagnostics() const override;
        bool selfTest() override;

        bool readChannels(RCData& data);
        int16_t getChannel(int index) const;
        int getChannelNum() const;

        bool isConnected(int timeout = 100) const; 
        bool isFailsafe() const;
        int64_t getTimeSinceLastUpdate() const;
        RCStatistics getRCStatistics() const;

        using RCCallback = std::function<void(const RCData&)>;
        void setCallback(RCCallback callback);  

    private:
        void threadFunc();
        void updateLatestData(const RCData& data);

        std::unique_ptr<RCBase> _rc_receiver;
        std::unique_ptr<std::thread> _thread;
        std::atomic<bool> _running;
        std::atomic<SensorStatus> _status;
        
        mutable std::mutex _error_mutex;
        std::string _last_error;

        mutable std::mutex _data_mutex;
        RCData _latest_data;
        bool _data_valid;

        std::mutex _callback_mutex;
        RCCallback _callback;

        std::atomic<int64_t> _update_count;
        std::atomic<int64_t> _error_count;
        std::chrono::steady_clock::time_point _start_time;
        const std::string _name = "SerialRCPlugin";        
};

}
}
