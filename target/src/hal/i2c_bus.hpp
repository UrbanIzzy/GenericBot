/**
  ******************************************************************************
  * @file    i2c_bus.c
  * @author  UrbanIzzy
  * @date    Feb 01, 2026
  * @brief   This file contain class headers of I2C bus management
  ******************************************************************************
*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <string>
#include <mutex>
#include <chrono>

namespace robotics {
namespace hal{

/* Defines -------------------------------------------------------------------*/
struct I2CBusConfig {
    uint32_t timeout_ms = 1000;
    uint32_t retries = 3;
};

struct I2CBusState {
    uint64_t total_tarnsactions = 0;
    uint64_t failed_transactions = 0;
    uint64_t total_retries = 0;
    uint64_t bus_recoveries = 0;
};

/* Classes -------------------------------------------------------------------*/
class I2CBus {
    public:
        bool open(int bus_num, const I2CBusConfig& config = {});
        void close();
        bool isOpen() const { return _fd >= 0; }

        int getBusNum() const { return _bus_num; }
        const I2CBusState& getState() const;
        void resetState();
        std::string getLastError() const { return _last_error;}

    protected:
        I2CBus();
        explicit I2CBus(int bus_num);
        virtual ~I2CBus();
        
        bool recover();
        bool recoverBusInternal();
        bool setSlaveAddr(uint8_t addr);
        int readData(uint8_t* data, size_t len);
        int writeData(const uint8_t* data, size_t len);
        bool writeReadData(const uint8_t* wdata, size_t wlen, uint8_t* rdata, size_t rlen);

        std::mutex& getMutex() { return _bus_mutex; }
    
    private:
        int _fd;
        int _bus_num;
        uint8_t _curr_addr;
        bool _addr_set;
        
        std::string _dev_path;
        std::string _last_error;

        I2CBusConfig _config;
        I2CBusState _state;


        mutable std::mutex _bus_mutex;

        bool _apply_configuration();
        // void _updteErrorState();
};

} // namespace hal
} // namespace robotics