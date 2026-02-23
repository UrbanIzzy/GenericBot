/**
 * @file pwm_dev_linux.cpp
 * @brief Linux sysfs PWM device implementation
 * */

#include "pwm_dev_linux.hpp"
#include "../logger.hpp"
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <sys/stat.h>

namespace robotics {
namespace hal {

LinuxPWMDev::LinuxPWMDev(int chip_num, int channel_num)
    : _chip_num(chip_num)
    , _channel_num(channel_num)
    , _exported(false)
    , _initialized(false) {
        _config.period_ns = 0;
        _config.duty_ns = 0;
        _config.polarity_inverted = false;
        _config.enabled = false;

        LOG_DEBUG("LinuxPWMDev created for chip %d channel %d", _chip_num, _channel_num);
}

LinuxPWMDev::~LinuxPWMDev() {
    deinit();
}

bool LinuxPWMDev::initialize() {
    std::lock_guard<std::mutex> lock(_mutex);

    if(_initialized) {
        LOG_WARN("LinuxPWMDev already initialized");
        return false;
    }

    if(!exportChannel()) {
        _last_error = "Failed to export channel";
        return false;
    }

    usleep(100000); // Wait for sysfs to create the filesu

    if(!isExported()) {
        _last_error = "Channel not exported after export attempt";
        return false;
    }

    if(!disable()) {
        _last_error = "Failed to disable channel during initialization";
        return false;
    }
    
    _initialized = true;
    LOG_INFO("LinuxPWMDev initialized for chip %d channel %d", _chip_num, _channel_num);
    return true;
}

void LinuxPWMDev::deinit() {
    std::lock_guard<std::mutex> lock(_mutex);

    if(!_initialized) {
        return;
    }

    disable();
    unexportChannel();
    _initialized = false;
    _exported = false;
    LOG_INFO("LinuxPWMDev deinitialized for chip %d channel %d", _chip_num, _channel_num);
}

bool LinuxPWMDev::exportChannel() {
    if(isExported()) {
        return true;
    }

    std::string export_path = getChipPath() + "/export";
    std::ofstream export_file(export_path);
    if(!export_file.is_open()) {
        _last_error = "Failed to open export file: " + export_path;
        return false;
    }

    export_file << _channel_num;
    export_file.close();
    if(export_file.fail()) {
        _last_error = "Failed to write to export file: " + export_path;
        return false;
    }

    _exported = true;
    return true;
}

void LinuxPWMDev::unexportChannel() {
    if(!_exported) {
        return;
    }

    std::string unexport_path = getChipPath() + "/unexport";
    std::ofstream unexport_file(unexport_path);

    if(unexport_file.is_open()) {
        unexport_file << _channel_num;
        unexport_file.close();
        LOG_DEBUG("Unexported channel %d from chip %d", _channel_num, _chip_num);
    }
    _exported = false;
}

bool LinuxPWMDev::isExported() const {
    struct stat st;
    return stat(getPWMPath().c_str(), &st) == 0;
}


bool LinuxPWMDev::setPeriod(uint32_t period_ns) {
    std::lock_guard<std::mutex> lock(_mutex);
    if(!_initialized) {
        _last_error = "Device not initialized";
        return false;
    }

    if(period_ns < _config.duty_ns) {
        LOG_DEBUG("Adjusting duty cycle from %u ns to fit new period %u ns", _config.duty_ns, period_ns);
        if(!writeFile("duty_cycle", std::to_string(period_ns))) {
            return false;
        }
        _config.duty_ns = period_ns;
    }
    
    if(!writeFile("period", std::to_string(period_ns))) {
        return false;
    }
    _config.period_ns = period_ns;

    LOG_DEBUG("Set period to %u ns for chip %d channel %d", period_ns, _chip_num, _channel_num);
    return true;
}   

bool LinuxPWMDev::setDutyCycle(uint32_t duty_ns) {
    std::lock_guard<std::mutex> lock(_mutex);
    if(!_initialized) {
        _last_error = "Device not initialized";
        return false;
    }

    if(_config.period_ns > 0 && duty_ns > _config.period_ns) {
        _last_error = "Duty cycle cannot be greater than period";
        return false;
    }

    if(!writeFile("duty_cycle", std::to_string(duty_ns))) {
        _last_error = "Failed to write duty cycle";
        return false;
    }
    _config.duty_ns = duty_ns;

    LOG_DEBUG("Set duty cycle to %u ns for chip %d channel %d", duty_ns, _chip_num, _channel_num);
    return true;
}

bool LinuxPWMDev::setDutyCyclePercent(float duty_percent) {
    if(duty_percent < 0.0f || duty_percent > 100.0f) {
        _last_error = "Duty cycle percent must be between 0 and 100";
        return false;
    }   

    std::lock_guard<std::mutex> lock(_mutex);
    if(_config.period_ns == 0) {
        _last_error = "Period must be set before setting duty cycle percent";
        return false;
    }
    uint32_t duty_ns = static_cast<uint32_t>((duty_percent / 100.0f) * _config.period_ns);

    _mutex.unlock();
    bool result = setDutyCycle(duty_ns);
    _mutex.lock();
    return result;
} 

bool LinuxPWMDev::setDutyCycleMicro(uint32_t duty_us) {
    uint32_t duty_ns = duty_us * 1000;
    return setDutyCycle(duty_ns);
}

bool LinuxPWMDev::setPolarity(bool inverted) {
    std::lock_guard<std::mutex> lock(_mutex);
    if(!_initialized) {
        _last_error = "Device not initialized";
        return false;
    }

    bool was_enabled = _config.enabled;
    if(was_enabled) {
        _mutex.unlock();
        disable();
        _mutex.lock();
    }

    std::string polarity_value = inverted ? "inversed" : "normal";
    if(!writeFile("polarity", polarity_value)) {
        _last_error = "Failed to write polarity";
        return false;
    }
    _config.polarity_inverted = inverted;

    if(was_enabled) {
        _mutex.unlock();
        enable();
        _mutex.lock();
    }
    LOG_DEBUG("Set polarity to %s for chip %d channel %d", polarity_value.c_str(), _chip_num, _channel_num);
    return true;
}


bool LinuxPWMDev::enable() {
    std::lock_guard<std::mutex> lock(_mutex);
    if(!_initialized) {
        _last_error = "Device not initialized";
        return false;
    }

    if(_config.enabled) {
        return true;
    }

    if(!writeFile("enable", "1")) {
        _last_error = "Failed to enable PWM";
        return false;
    }
    _config.enabled = true;
    LOG_DEBUG("Enabled PWM for chip %d channel %d", _chip_num, _channel_num);
    return true;
}

bool LinuxPWMDev::disable() {
    std::lock_guard<std::mutex> lock(_mutex);
    if(!_initialized) {
        _last_error = "Device not initialized";
        return false;
    }

    if(!_config.enabled) {
        return true;
    }

    if(!writeFile("enable", "0")) {
        _last_error = "Failed to disable PWM";
        return false;
    }
    _config.enabled = false;
    LOG_DEBUG("Disabled PWM for chip %d channel %d", _chip_num, _channel_num);
    return true;
}

bool LinuxPWMDev::isEnabled() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _config.enabled;
}

PWMConfig LinuxPWMDev::getConfig() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _config;
}

std::string LinuxPWMDev::getLastError() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _last_error;
}


std::string LinuxPWMDev::getChipPath() const {
    std::ostringstream oss;
    oss << "/sys/class/pwm/pwmchip" << _chip_num;
    return oss.str();
}

std::string LinuxPWMDev::getPWMPath() const {
    std::ostringstream oss;
    oss << getChipPath() << "/pwm" << _channel_num;
    return oss.str();
}

bool LinuxPWMDev::writeFile(const std::string& file_path, const std::string& value) {
    std::string path = getPWMPath() + "/" + file_path;
    
    std::ofstream file(path);
    if(!file.is_open()) {
        _last_error = "Failed to open file: " + path;
        return false;
    }
    file << value;
    file.close();

    if(file.fail()) {
        _last_error = "Failed to write to file: " + path;
        return false;
    } 
    return true;
}

bool LinuxPWMDev::readFile(const std::string& file_path, std::string& value) {
    std::string path = getPWMPath() + "/" + file_path;

    std::ifstream file(path);
    if(!file.is_open()) {
        _last_error = "Failed to open file: " + path;
        return false;
    }
    std::getline(file, value);
    file.close();

    if(file.fail()) {
        _last_error = "Failed to read from file: " + path;
        return false;
    }
    return true;
} 
}
}