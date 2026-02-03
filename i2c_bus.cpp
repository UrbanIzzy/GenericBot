/**
 * @file i2c_bus.cpp
 * @brief MPU9250 9-axis IMU sensor driver implementation
 */

 /* Includes ------------------------------------------------------------------*/
#include "i2c_bus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <cstring>
#include <iostream>
#include <map>

/* Linux I2C definitions ------------------------------------------------------*/
#ifndef I2C_FUNC_I2C
#define I2C_FUNC_I2C 0x00000001
#endif

#ifndef I2C_M_RD
#define I2C_M_RD 0x0001
#endif

namespace robotics {
namespace hal{
/* Static Members ------------------------------------------------------------*/
static std::map<int, int> s_fd_map;
static std::map<int, int> s_count_map;
static std::mutex s_bus_map_mutex;

/* Public Methods ------------------------------------------------------------*/
I2CBus::I2CBus() 
    : _fd(-1)
    , _bus_num(-1) 
    , _curr_addr(0xFF)
    , _addr_set(false)
    {
        std::memset(&_state, 0, sizeof(_state));
    }

I2CBus::I2CBus(int bus_num)
    : _fd(-1)
    , _bus_num(bus_num) {}

I2CBus::~I2CBus() {
    close();
}

bool I2CBus::open(int bus_num, const I2CBusConfig& config){
    std::lock_guard<std::mutex> lock(s_bus_map_mutex);

    auto it = s_fd_map.find(bus_num); 
    if(it != s_fd_map.end()){
        _fd - it->second;
        _bus_num = bus_num;
        _dev_path = "/dev/i2c-" + std::to_string(bus_num);
        _config = config;

        s_count_map[bus_num]++;

        std::cout << "I2C Bus " << bus_num << " already open. Reusing file descriptor." << std::endl;
        return true;
    }

    _dev_path = "/dev/i2c-" + std::to_string(bus_num);
    int fd = ::open(_dev_path.c_str(), O_RDWR);
    if(fd < 0){
        _last_error = "Failed to open I2C bus " + std::to_string(bus_num) + ": " + std::strerror(errno);
        return false;
    }   
    _config = config;

    unsigned long timeout = _config.timeout_ms / 10;
    if(timeout == 0) timeout = 1;
    if(ioctl(fd, I2C_TIMEOUT, timeout) < 0){
        _last_error = "Failed to set I2C timeout: " + std::string(std::strerror(errno));
        ::close(fd);
        return false;
    }

    if(ioctl(fd, I2C_RETRIES, _config.timeout_ms) < 0){
        _last_error = "Failed to set I2C retries: " + std::string(std::strerror(errno));
        ::close(fd);
        return false;
    }

    s_fd_map[bus_num] = fd;
    s_count_map[bus_num] = 1;
    _fd = fd;
    _bus_num = bus_num;

    std::cout << "I2C Bus " << bus_num << " opened successfully." << std::endl;
    return true;
}   

void I2CBus::close(){
    if(_fd < 0) return;
    std::lock_guard<std::mutex> lock(s_bus_map_mutex);

    auto ref_it = s_count_map.find(_bus_num);
    if(ref_it == s_count_map.end()){
        _fd = -1;
        std::cout << "I2C Bus " << _bus_num << " not found in Register map." << std::endl;
        return;
    }

    ref_it->second--;

    std::cout << "I2C Bus " << _bus_num << " close called. Remaining references: " << ref_it->second << std::endl;

    if(ref_it->second == 0){
        auto fd_it = s_fd_map.find(_bus_num);
        if(fd_it != s_fd_map.end()){
            int tmp_fd = fd_it->second;
            s_fd_map.erase(fd_it);
            s_count_map.erase(_bus_num);
            if(tmp_fd >= 0){
                ::close(tmp_fd);
            }
        }
        else{
            s_fd_map.erase(fd_it);
        }
    }
    _fd = -1;
    _bus_num = -1;
}

I2CBusState I2CBus::getState() const {
    std::lock_guard<std::mutex> lock(_bus_mutex);
    return _state; 
}

void I2CBus::resetState(){
    std::lock_guard<std::mutex> lock(_bus_mutex);
    
    std::memset(&_state, 0, sizeof(_state));
}

bool I2CBus::setSlaveAddr(uint8_t addr){
    std::lock_guard<std::mutex> lock(_bus_mutex);

    if(_fd < 0){
        _last_error = "I2C bus not open";
        return false;
    }

    if(_addr_set  && (_curr_addr == addr)){
        return true;
    }

    if(ioctl(_fd, I2C_SLAVE, addr) < 0){
        _last_error = "Failed to set I2C slave address to " + std::to_string(addr) + ": " + std::strerror(errno);
        return false;
    }
    _curr_addr = addr;
    _addr_set = true;
    return true;
}   

int I2CBus::readData(uint8_t* data, size_t len){
    std::lock_guard<std::mutex> lock(_bus_mutex);

    if(_fd < 0){
        _last_error = "I2C bus not open";
        return -1;
    }

    _state.total_tarnsactions++;
    ssize_t ret = ::read(_fd, data, len);
    if(ret < 0){
        _last_error = "Failed to read data from I2C bus: " + std::string(std::strerror(errno));
        _state.failed_transactions++;

        if(errno == EIO || errno == ENXIO){
            if(recoverBusInternal()){
                ret = ::read(_fd, data, len);
                if(ret >= 0){
                    _state.total_retries++;
                }
            }
        }
    } else if(ret != static_cast<ssize_t>(len)){
        _last_error = "Incomplete read from I2C bus. Expected " + std::to_string(len) + " bytes, got " + std::to_string(ret) + " bytes.";
        _state.failed_transactions++;
        return -1;
    }
    return static_cast<int>(ret);
}

int I2CBus::writeData(const uint8_t* data, size_t len){
    std::lock_guard<std::mutex> lock(_bus_mutex);

    if(_fd < 0){
        _last_error = "I2C bus not open";
        return -1;
    }
    
    _state.total_tarnsactions++;
    ssize_t ret = ::write(_fd, data, len);
    if(ret < 0){
        _last_error = "Failed to write data to I2C bus: " + std::string(strerror(errno));
        _state.failed_transactions++;

        if(errno == EIO || errno == ENXIO){
            if(recoverBusInternal()){
                ret = ::write(_fd, data, len);
                if(ret >= 0){
                    _state.total_retries++;
                }
            }
        }
    }
    else if(ret != static_cast<ssize_t>(len)){
        _last_error = "Incomplete write to I2C bus. Expected " + std::to_string(len) + " bytes, got " + std::to_string(ret) + " bytes.";
        _state.failed_transactions++;
        return -1;
    }
    return static_cast<int>(ret);
}

bool I2CBus::writeReadData(const uint8_t* wdata, size_t wlen, uint8_t* rdata, size_t rlen){
    std::lock_guard<std::mutex> lock(_bus_mutex);

    if(_fd < 0){
        _last_error = "I2C bus not open";
        return false;
    }

    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    messages[0].addr = _curr_addr;
    messages[0].flags = 0;
    messages[0].len = wlen;
    messages[0].buf = const_cast<uint8_t*>(wdata);

    messages[1].addr = _curr_addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len = rlen;
    messages[1].buf = rdata;

    packets.msgs = messages;
    packets.nmsgs = 2;

    _state.total_tarnsactions++;
    if(ioctl(_fd, I2C_RDWR, &packets) < 0){
        _last_error = "Failed to perform I2C write-read transaction: " + std::string(strerror(errno));
        _state.failed_transactions++;

        if(errno == EIO || errno == ENXIO){
            if(recoverBusInternal()){
                if(ioctl(_fd, I2C_RDWR, &packets) >= 0){
                    _state.total_retries++;
                    return true;
                }
            }
        }
        return false;
    }
    return true;
}

bool I2CBus::recover(){
    std::lock_guard<std::mutex> lock(_bus_mutex);
    return recoverBusInternal();
}

bool I2CBus::recoverBusInternal(){
 
    std::cout << "Attempting I2C bus recovery for bus " << _bus_num << std::endl;
    _state.bus_recoveries++;

    int fd = _fd;
    usleep(10000);

    int n_fd = ::open(_dev_path.c_str(), O_RDWR);
    if(n_fd < 0){
        _last_error = "Failed to reopen I2C bus during recovery: " + std::string(std::strerror(errno));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(s_bus_map_mutex);
        s_fd_map[_bus_num] = n_fd;
    }
    ::close(fd);
    _fd = n_fd;

    unsigned long timeout = _config.timeout_ms / 10;
    if(timeout == 0) timeout = 1;

    if(ioctl(_fd, I2C_TIMEOUT, timeout) < 0 ||
        ioctl(_fd, I2C_RETRIES, _config.retries) < 0){
        _last_error = "Failed to reapply I2C configuration during recovery: " + std::string(std::strerror(errno));
        // _fd = -1;
        return false;
    }

    _addr_set = false;
    std::cout << "I2C bus " << _bus_num << " recovery successful." << std::endl;
    return true;
}

}   // namespace hal
}   // namespace robotics


