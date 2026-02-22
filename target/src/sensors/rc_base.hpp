/**
  ******************************************************************************
  * @file    rc_base.c
  * @author  UrbanIzzy
  * @date    Feb 17, 2026
  * @brief   Abstract base class for RC receivers
  ******************************************************************************
*/

#pragma once

#include "../hal/uart_dev.hpp"

#include <memory>
#include <chrono>
#include <array>
#include <string>

namespace robotics {
namespace sensors {

namespace RCDefs{
        constexpr uint16_t CHANNEL_MIN = 800;
        constexpr uint16_t CHANNEL_MAX = 2200; 
        constexpr uint16_t CHANNEL_DEFAULT = 1500;
        constexpr int MAX_SYNC_ATTEMPTS = 100;
        constexpr size_t PACKET_SIZE = 32;
        constexpr int CHANNEL_NUM = 14;
        constexpr uint8_t HEADER_BYTE1 = 0x20;
        constexpr uint8_t HEADER_BYTE2 = 0x40;
}

struct RCData {
    std::array<uint16_t, RCDefs::CHANNEL_NUM> channels;
    uint8_t channel_num;
    bool failsafe;
    bool frame_lost;
    std::chrono::steady_clock::time_point timestamp;

    RCData() {
        channels.fill(RCDefs::CHANNEL_DEFAULT);    // Default to neutral PWM
        channel_num = 0;        // Default to 8 channels
        failsafe = false;
        frame_lost = false;
        timestamp = std::chrono::steady_clock::now();
    }

    void setChannel(int index, uint16_t value) {
        if(index >= 0 && index < RCDefs::CHANNEL_NUM) {
            if(value < RCDefs::CHANNEL_MIN) value = RCDefs::CHANNEL_MIN;
            if(value > RCDefs::CHANNEL_MAX) value = RCDefs::CHANNEL_MAX;
            channels[index] = value;
        }
    }
};

struct RCStatistics {
    uint64_t total_packets = 0;
    uint64_t valid_packets = 0;
    uint64_t invalid_packets = 0;
    uint64_t lost_frames = 0;
    uint64_t failsafe_events = 0;

    std::chrono::steady_clock::time_point first_packet_time;
    std::chrono::steady_clock::time_point last_packet_time;

    float getSuccessRate() const {
        if(total_packets == 0) return 0.0f;
        return static_cast<float>(valid_packets) / static_cast<float>(total_packets);
    }

    float getPacketRate() const {
        if(total_packets < 2) return 0.0f;

        auto duration = std::chrono::duration_cast<std::chrono::seconds>(last_packet_time - first_packet_time);
        
        if(duration.count() == 0) return 0.0f;

        return static_cast<float>(total_packets * 1000.0f) / static_cast<float>(duration.count());
    }
};

class RCBase {
    public:
        virtual ~RCBase() = default;

        virtual bool initialize() = 0;
        virtual bool readChannels(RCData& data) = 0;
        virtual int getChannelNum() const = 0;
        virtual std::string getProtocolName() const = 0;

        bool isConnected(int timeout = 100) const {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_update_time);

            return duration.count() < timeout; // Consider connected if received packet within last 1 second
        }

        int64_t getTimeSinceLastUpdate() const {
            auto now = std::chrono::steady_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_update_time);
            return duration.count();
        }

        std::chrono::steady_clock::time_point getLastUpdateTime() const {
            return _last_update_time;
        }

        std::string getLastError() const {
            return _last_error;
        }

        RCStatistics getStatistics() const {
            return _stat;
        }

        void resetStatistic() {
            _stat = RCStatistics();
        }

        std::string getDevicePath() const {
            return _dev_path;
        }   
        
    protected:
        std::unique_ptr<hal::UARTDev> _uart_dev;
        std::string _dev_path;
        std::chrono::steady_clock::time_point _last_update_time;
        std::string _last_error;
        RCStatistics _stat;

        void updateStatsValid(){
            _stat.total_packets++;
            _stat.valid_packets++;
            _stat.last_packet_time = std::chrono::steady_clock::now();

            if(_stat.total_packets == 1){
                _stat.first_packet_time = _stat.last_packet_time;
            }
            _last_update_time = _stat.last_packet_time;
        }

        void updateStatsInvalid(){
            _stat.total_packets++;
            _stat.invalid_packets++;
            _last_update_time = std::chrono::steady_clock::now();
        }

        void updateStatsLostFrame(){
            _stat.lost_frames++;
        }

        void updateStatsFailsafe(){
            _stat.failsafe_events++;
        }

        uint16_t calcChecksum16(const uint8_t* data, size_t len) const {
            uint16_t checksum = 0xFFFF;
            for(size_t i = 0; i < len; i++) {
                checksum += data[i];
            }
            return checksum;
        }

        uint8_t calcChecksumXOR(const uint8_t* data, size_t len) const {
            uint8_t checksum = 0;
            for(size_t i = 0; i < len; i++) {
                checksum ^= data[i];
            }
            return checksum;
        }
        
        int16_t clampChannel(int16_t value) const {
            if(value < RCDefs::CHANNEL_MIN) return RCDefs::CHANNEL_MIN;
            if(value > RCDefs::CHANNEL_MAX) return RCDefs::CHANNEL_MAX;
            return value;
        }
};


}   // namespace sensors
}   // namespace robotics