/**
  ******************************************************************************
  * @file    uart_dev.c
  * @author  UrbanIzzy
  * @date    Feb 17, 2026
  * @brief    UART/Serial Hardware Abstraction Layer implementation
  ******************************************************************************
*/

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <mutex>

namespace robotics {
namespace hal {

enum class BaudRate {
    BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
    BAUD_57600 = 57600,
    BAUD_115200 = 115200,
    BAUD_230400 = 230400,
    BAUD_460800 = 460800
    // Add more baud rates as needed
};

enum class Parity {
    NONE,
    EVEN,
    ODD
};

enum class StopBits {
    ONE,
    TWO
};

enum class DataBits {
    FIVE = 5,
    SIX = 6,
    SEVEN = 7,
    EIGHT = 8
};

enum class FlowControl {
    NONE,
    RTS_CTS,    // Hardware flow control
    XON_XOFF    // Software flow control
};

struct UARTConfig {
    BaudRate baud_rate = BaudRate::BAUD_115200;
    Parity parity = Parity::NONE;
    StopBits stop_bits = StopBits::ONE;
    DataBits data_bits = DataBits::EIGHT;
    FlowControl flow_control = FlowControl::NONE;
    bool blocking = true;  // Blocking mode by default
    std::chrono::milliseconds read_timeout_ms{100}; 
};

class UARTDev {
    public:
        static std::unique_ptr<UARTDev> open(const std::string& dev_path, const UARTConfig& config = {});
        ~UARTDev();


        UARTDev(const UARTDev&) = default;
        UARTDev& operator=(const UARTDev&) = default;
        UARTDev(UARTDev&&) noexcept;
        UARTDev& operator=(UARTDev&&) noexcept;

        size_t read(uint8_t* data, size_t size);
        bool readExact(uint8_t* data, size_t size);
        std::vector<uint8_t> readUntil(uint8_t delimiter, size_t max_size = 1024);
        std::string readLine(size_t max_size = 1024);
        size_t write(const uint8_t* data, size_t size);
        size_t write(const std::string& data);
        size_t available();
        void flush();
        void flushInput();
        void flushOutput();

        bool setBaudRate(BaudRate baud_rate);
        void setTimeout(std::chrono::milliseconds timeout);
        std::string geDevice() const { return _dev; };
        std::string getLastError() const { return _last_error; }
        bool isOpen() const { return _fd >= 0; }

    private:
        UARTDev(int fd, const std::string& dev_path, const UARTConfig& config);
        bool setConfig(const UARTConfig& config);
        int getBaudRate(BaudRate baud_rate);
        
        int _fd;
        std::string _dev;
        std::string _last_error;
        UARTConfig _config;
        mutable std::mutex _mutex;
};

} // namespace hal
} // namespace robotics