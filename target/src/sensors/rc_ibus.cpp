/**
 * @file rc_ibus.cpp
 * @brief FlySky IBUS protocol receiver implementation
 */

 /* Includes ------------------------------------------------------------------*/
#include "rc_ibus.hpp"
#include "../logger.hpp"
#include <cstring>

namespace robotics {
namespace sensors {

IBUS_Reciver::IBUS_Reciver(const std::string& dev)
    : _is_synced(false) {
    _dev_path = dev;
    std::memset(_packet_buffer, 0, sizeof(_packet_buffer));
}

bool IBUS_Reciver::initialize() {
    LOG_INFO("%s", "Initializing IBUS Receiver on device: %s", _name.c_str(), _dev_path.c_str());
    
    hal::UARTConfig config;
    config.baud_rate = hal::BaudRate::BAUD_115200;
    config.parity = hal::Parity::NONE;
    config.stop_bits = hal::StopBits::ONE;
    config.data_bits = hal::DataBits::EIGHT;
    config.flow_control = hal::FlowControl::NONE;
    config.blocking = true;
    config.read_timeout_ms = std::chrono::milliseconds(100);

    _uart_dev = hal::UARTDev::open(_dev_path, config);
    if(!_uart_dev){
        _last_error = "Failed to open UART device: " + _dev_path;
        LOG_INFO("%s", "%s", _name.c_str(), _last_error.c_str());
        return false;
    }
    LOG_INFO("%s", "UART opened successfully", _name.c_str());

    _uart_dev->flush();

    if(!sync()){
        _last_error = "Failed to synchronize with IBUS data stream";
        LOG_ERROR("%s", "%s", _name.c_str(), _last_error.c_str());
        //return false; no returning false here since sync can be achived later during read
    }
    LOG_INFO("%s", "Initialization complete", _name.c_str());
    return true;
}

bool IBUS_Reciver::sync() {
    LOG_DEBUG("%s", "Attempting to synchronize with IBUS data stream...", _name.c_str());
    uint8_t byte1, byte2;
    int attempts = 0;

    while (attempts < RCDefs::MAX_SYNC_ATTEMPTS) {
        if(_uart_dev->read(&byte1, 1) && _uart_dev->readExact(&byte2, 1)){
            if (byte1 == RCDefs::HEADER_BYTE1 && byte2 == RCDefs::HEADER_BYTE2) {
                _is_synced = true;
                LOG_DEBUG("%s", "Synchronization successful after %d attempts", _name.c_str(), attempts + 1);
                return true;
            }
        }
        attempts++;
    }
    LOG_WARN("%s", "Failed to synchronize after %d attempts", _name.c_str(), RCDefs::MAX_SYNC_ATTEMPTS);
    return false;
}

bool IBUS_Reciver::readPacket(uint8_t* data) {
    if(!_is_synced){
        if(!sync()){
            return false;
        }
        
        size_t r_bytes = _uart_dev->readExact(data + 2, RCDefs::PACKET_SIZE - 2);
        return r_bytes == (RCDefs::PACKET_SIZE - 2);
    }

    size_t r_bytes = _uart_dev->readExact(data, RCDefs::PACKET_SIZE);
    if(r_bytes != RCDefs::PACKET_SIZE){
        LOG_DEBUG("%s", "incomplete packet: received %zu/ %zu bytes", _name.c_str(), r_bytes, RCDefs::PACKET_SIZE);
        _is_synced = false; // Lost sync, need to resync
        return false;
    }
    return true;
}

bool IBUS_Reciver::validatePacket(const uint8_t* packet) const {
    if(packet[0] != RCDefs::HEADER_BYTE1 || packet[1] != RCDefs::HEADER_BYTE2){
        LOG_DEBUG("%s", "Invalid header bytes: 0x%02X 0x%02X", _name.c_str(), packet[0], packet[1]);
        return false;
    }

    uint16_t received_checksum = (packet[30] << 8) | packet[31];
    uint16_t calculated_checksum = calcIBUSChecksum(packet, RCDefs::PACKET_SIZE - 2);

    if(received_checksum != calculated_checksum){
        LOG_DEBUG("%s", "Checksum mismatch: received 0x%04X, calculated 0x%04X", _name.c_str(), received_checksum, calculated_checksum);
        return false;
    }
    return true;
}

uint16_t IBUS_Reciver::calcIBUSChecksum(const uint8_t* data, size_t len) const {
    uint16_t checksum = 0xFFFF;
    for(size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

int16_t IBUS_Reciver::extractChannelValue(const uint8_t* data, int index) const {
    if(index < 0 || index >= RCDefs::CHANNEL_NUM) return RCDefs::CHANNEL_DEFAULT;
    int16_t value = (data[2 + index * 2] << 8) | data[3 + index * 2];
    return value;
}

bool IBUS_Reciver::parsePacket(const uint8_t* packet, RCData& data) {
    for(int i = 0; i < RCDefs::CHANNEL_NUM; i++){
        int16_t raw_value = extractChannelValue(packet, i);
        data.setChannel(i, clampChannel(raw_value));
    }
    data.channel_num = RCDefs::CHANNEL_NUM;
    data.timestamp = std::chrono::steady_clock::now();

    bool all_center = true;
    for(int i = 0; i < RCDefs::CHANNEL_NUM; i++){
        if(data.channels[i] != RCDefs::CHANNEL_DEFAULT){
            all_center = false;
            break;
        }
    }
    data.failsafe = all_center;
    data.frame_lost = false;
    return true;
}

bool IBUS_Reciver::readChannels(RCData& data) {
    if(!readPacket(_packet_buffer)){
        updateStatsInvalid();
        _is_synced = false;
        return false;
    }

    if(!validatePacket(_packet_buffer)){
        updateStatsInvalid();
        _is_synced = false;
        return false;
    }

    if(!parsePacket(_packet_buffer, data)){
        updateStatsInvalid();
        _last_error = "Failed to parse IBUS packet";
        return false;
    }

    updateStatsValid();

    if(data.failsafe){
        updateStatsFailsafe();
        LOG_WARN("%s", "Failsafe condition detected (all channels at center)", _name.c_str());
    }

    if(_stat.valid_packets % 1000 == 0){ // Log stats every 1000 valid packets
        LOG_INFO("%s", "Stats: %lu valid, %llu invalid, %.2f%% success, %.2f pkt/s", 
            _name.c_str(), _stat.valid_packets, _stat.invalid_packets, _stat.getSuccessRate() * 100.0f, _stat.getPacketRate());
    }
    return true;
}   
}
}