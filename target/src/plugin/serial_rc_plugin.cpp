/**
 * @file serial_rc_plugin.cpp
 * @brief RC receiver plugin implementation with background threading
 * */

#include "serial_rc_plugin.hpp"
#include "../logger.hpp"
#include "../sensors/rc_ibus.hpp"
#include <sstream>


namespace robotics {
namespace sensors { 

std::unique_ptr<SerialRCPlugin> SerialRCPlugin::create(const std::string& config){
    std::istringstream ss(config);
    std::string protocol, dev_path, baud_str;

    if(!std::getline(ss, protocol, ':') || 
       !std::getline(ss, dev_path, ':') || 
       !std::getline(ss, baud_str)){
        LOG_ERROR("Invalid config format. Expected: protocol:dev_path:baud_rate");
        return nullptr;
    }

    std::unique_ptr<RCBase> receiver;
    if(protocol == "ibus"){
        receiver = std::make_unique<IBUS_Reciver>(dev_path);
    } else {
        LOG_ERROR("Unsupported protocol: %s", protocol.c_str());
        return nullptr;
    }
    return std::make_unique<SerialRCPlugin>(std::move(receiver));
}

SerialRCPlugin::SerialRCPlugin(std::unique_ptr<RCBase> rc_receiver)
    : _rc_receiver(std::move(rc_receiver))
    , _running(false)
    , _status(SensorStatus::UNINITIALIZED)
    , _data_valid(false)
    , _update_count(0)
    , _error_count(0) {
        LOG_DEBUG("SerialRCPlugin created with RC receiver: %s", _rc_receiver->getProtocolName().c_str());
}

SerialRCPlugin::~SerialRCPlugin() {
    stop();
    LOG_DEBUG("SerialRCPlugin destroyed");
}

bool SerialRCPlugin::initialize() {
    LOG_INFO("Initializing SerialRCPlugin with receiver: %s", _rc_receiver->getProtocolName().c_str());


    if(!_rc_receiver){
        std::lock_guard<std::mutex> lock(_error_mutex);
        _last_error = "RC receiver instance is null";
        LOG_ERROR("%s", _last_error.c_str());
        return false;
    }        
    
    if(!_rc_receiver->initialize()){
        std::lock_guard<std::mutex> lock(_error_mutex);
        _last_error = "Failed to initialize RC receiver: " + _rc_receiver->getLastError();
        LOG_ERROR("%s", _last_error.c_str());
        _status = SensorStatus::ERROR;
        return false;
    }

    _status = SensorStatus::INITIALIZED;
    LOG_INFO("Initialization successful"); 
    return true;
}

bool SerialRCPlugin::start() {
    if(_status == SensorStatus::RUNNING){
        LOG_ERROR("Already running");
        return true;
    }

    if(_status != SensorStatus::INITIALIZED){
        _last_error = "Must initialize before starting";
        LOG_ERROR("%s", _last_error.c_str());
        return false;
    }
    LOG_INFO("Starting SerialRCPlugin...");

    _running = true;
    _start_time = std::chrono::steady_clock::now();

    try {
        _thread = std::make_unique<std::thread>(&SerialRCPlugin::threadFunc, this);
        // TDB: Add thread priority setting here if needed
    } catch(const std::exception& e){
        std::lock_guard<std::mutex> lock(_error_mutex);
        _last_error = "Failed to start thread: " + std::string(e.what());
        LOG_ERROR("%s", _last_error.c_str());
        _running = false;
        _status = SensorStatus::ERROR;
        return false;
    }
    _status = SensorStatus::RUNNING;
    LOG_INFO("SerialRCPlugin started");
    return true;
}

void SerialRCPlugin::stop() {
    if(_status != SensorStatus::RUNNING){
      return;
    }

    LOG_INFO("Stopping SerialRCPlugin...");

    _running = false;
    if(_thread && _thread->joinable()){
        _thread->join();
        _thread.reset();
        LOG_INFO("SerialRCPlugin stopped");
    }
    _status = SensorStatus::STOPPED;
}

bool SerialRCPlugin::reset() {
    LOG_INFO("Resetting SerialRCPlugin...");

    stop();
    _update_count = 0;
    _error_count = 0;

    { 
        std::lock_guard<std::mutex> lock(_error_mutex);
        _data_valid = false;
    }

    return initialize() && start();
}

SensorMetadata SerialRCPlugin::getMetadata() const {
    SensorMetadata meta;
    
    meta.name = _name;
    meta.version = "1.0.0";
    meta.author = "UrbanIzzy";

    if(_rc_receiver){ 
        meta.capabilities = SensorCapability::NONE; // TDB: Set appropriate capabilities based on protocol
        meta.description += _rc_receiver->getProtocolName() + " protocol support";
    }
    return meta;
}

std::string SerialRCPlugin::getLastError() const {
    std::lock_guard<std::mutex> lock(_error_mutex);
    return _last_error;
}

bool SerialRCPlugin::setConfig(const std::string& key, const std::string& value) {
    key;
    value;  
    std::lock_guard<std::mutex> lock(_error_mutex);
    _last_error = "No configurable parameters supported for RC receiver";
    return false;
}

std::string SerialRCPlugin::getConfig(const std::string& key) const {
    (void)key;
    return "";
}

std::map<std::string, std::string> SerialRCPlugin::getDiagnostics() const {
    std::map<std::string, std::string> diag;
    if(_rc_receiver) {
        diag["Protocol"] = _rc_receiver->getProtocolName();
        diag["Channels"] = std::to_string(_rc_receiver->getChannelNum());
        diag["Device"] = _rc_receiver->getDevicePath();
    }

    switch(_status.load()){
        case SensorStatus::UNINITIALIZED: diag["Status"] = "uninitialized"; break;
        case SensorStatus::INITIALIZED:   diag["Status"] = "initialized"; break;
        case SensorStatus::RUNNING:       diag["Status"] = "running"; break;
        case SensorStatus::STOPPED:       diag["Status"] = "stopped"; break;
        case SensorStatus::ERROR:         diag["Status"] = "error"; break;
        default: diag["Status"] = "unknown"; break;
    }
    diag["Connected"] = isConnected() ? "Yes" : "No";
    diag["Failsafe"] = isFailsafe() ? "Yes" : "No";
    diag["TimeSinceLastUpdate_ms"] = std::to_string(getTimeSinceLastUpdate());

    diag["TotalUpdates"] = std::to_string(_update_count.load());
    diag["TotalErrors"] = std::to_string(_error_count.load());

    if(_rc_receiver){
        auto rc_stats = _rc_receiver->getStatistics();
        diag["TotalPackets"] = std::to_string(rc_stats.total_packets);
        diag["ValidPackets"] = std::to_string(rc_stats.valid_packets);
        diag["InvalidPackets"] = std::to_string(rc_stats.invalid_packets);

        float success_rate = rc_stats.getSuccessRate() * 100.0f;
        diag["SuccessRate"] = std::to_string(success_rate) + "%";

        float packet_rate = rc_stats.getPacketRate();
        diag["PacketRate"] = std::to_string(packet_rate) + " packets/sec";
    }
    
    {
        std::lock_guard<std::mutex> lock(_error_mutex);
        if(!_data_valid){
            for(int i = 0; i < std::min(4, static_cast<int>(_latest_data.channel_num)); i++){
                std::string key = "Channel" + std::to_string(i + 1);
                diag[key] = std::to_string(_latest_data.channels[i]);
            }
        }
    }
    return diag;
}

bool SerialRCPlugin::selfTest() {
    LOG_INFO("Performing self-test for SerialRCPlugin...");

    if(!_rc_receiver){
        LOG_ERROR("%s", _last_error.c_str());
        return false;
    }

    if(!_rc_receiver->initialize()){
        LOG_ERROR("%s", _last_error.c_str());
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(_error_mutex);
        if(!_data_valid){
            _last_error = "Failed to read channels during self-test: No valid data received";
            LOG_WARN("Self-Test failed (No valid data)");
            return false;
        }
    }
    LOG_INFO("Self-test passed for SerialRCPlugin");
    return true;
}

bool SerialRCPlugin::readChannels(RCData& data) {
    std::lock_guard<std::mutex> lock(_data_mutex);
    if(!_data_valid){
        return false;
    }

    data = _latest_data;
    return true;
}

int16_t SerialRCPlugin::getChannel(int index) const {
    std::lock_guard<std::mutex> lock(_data_mutex);

    if(!_data_valid || index < 0 || index >= _latest_data.channel_num){
        return -1; // Invalid channel value
    }
    return _latest_data.channels[index];
}

int SerialRCPlugin::getChannelNum() const {
    if(!_rc_receiver){
        return 0;
    }
    return _rc_receiver->getChannelNum();
}

bool SerialRCPlugin::isConnected(int timeout_ms) const {
    if(!_rc_receiver){
        return false;
    }
    return _rc_receiver->isConnected(timeout_ms);
}

bool SerialRCPlugin::isFailsafe() const {
    std::lock_guard<std::mutex> lock(_data_mutex);
    if(!_rc_receiver){
        return false;
    }
    return _latest_data.failsafe;
}

int64_t SerialRCPlugin::getTimeSinceLastUpdate() const {
    if(!_rc_receiver){
        return -1;
    }
    return _rc_receiver->getTimeSinceLastUpdate();
}

RCStatistics SerialRCPlugin::getRCStatistics() const {
    if(!_rc_receiver){
        return RCStatistics();
    }
    return _rc_receiver->getStatistics();
}

void SerialRCPlugin::setCallback(RCCallback callback) {
    std::lock_guard<std::mutex> lock(_callback_mutex);
    _callback = callback;
}

void SerialRCPlugin::threadFunc() {
    LOG_INFO("SerialRCPlugin thread started");

    while(_running){
        RCData data;
        if(_rc_receiver->readChannels(data)){
            updateLatestData(data);
            _update_count++;
        
            {
                std::lock_guard<std::mutex> lock(_callback_mutex);
                if(_callback){
                    try {
                        _callback(data);
                    } catch(const std::exception& e){
                        std::lock_guard<std::mutex> lock(_error_mutex);
                        _last_error = "Exception in callback: " + std::string(e.what());
                        LOG_ERROR("%s", _last_error.c_str());
                        _error_count++;
                    }
                }
            }

            if(_update_count % 1000 == 0){ // Log every 10 updates
                 LOG_INFO("Update count: %d, Error count: %d, success rate: %.2f%%", _update_count.load()
                    , _error_count.load()
                    , _rc_receiver->getStatistics().getSuccessRate() * 100.0f);
            }        
        } else {
            std::lock_guard<std::mutex> lock(_error_mutex);
            _last_error = "Failed to read channels: " + _rc_receiver->getLastError();
            LOG_WARN("%s", _last_error.c_str());
            _error_count++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Adjust sleep duration as needed
    }
    LOG_INFO("SerialRCPlugin thread exiting");
}

void SerialRCPlugin::updateLatestData(const RCData& data) {
    std::lock_guard<std::mutex> lock(_data_mutex);
    _latest_data = data;
    _data_valid = true;
}

}
}