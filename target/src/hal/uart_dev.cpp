/**
 * @file uart.hpp
 * @brief UART/Serial Hardware Abstraction Layer for Linux embedded systems
 */

#include "uart_dev.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termio.h>
#include <cstring>
#include <dirent.h>
#include <sys/select.h>

namespace robotics {
namespace hal {

std::unique_ptr<UARTDev> UARTDev::open(const std::string& device_path, const UARTConfig& config){
    int fd = ::open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0){
        return nullptr;
    }

    auto dev = std::unique_ptr<UARTDev>(new UARTDev(fd, device_path, config));
    if(!dev->setConfig(config)){
        return nullptr;
    }
    return dev;
}

UARTDev::UARTDev(int fd, const std::string& dev_path, const UARTConfig& config)
    : _fd(fd)
    , _dev(dev_path)
    , _config(config) {
}   

UARTDev::~UARTDev() {
    if(_fd >= 0){
        ::close(_fd);
    }
}

UARTDev::UARTDev(UARTDev&& other) noexcept
    : _fd(other._fd)
    , _dev(std::move(other._dev))
    , _last_error(std::move(other._last_error))
    , _config(std::move(other._config)) {
        other._fd = -1;
}

UARTDev& UARTDev::operator=(UARTDev&& other) noexcept {
    if(this != &other){
        if(_fd >= 0){
            ::close(_fd);
        }
        _fd = other._fd;
        _dev = std::move(other._dev);
        _last_error = std::move(other._last_error);
        _config = std::move(other._config);
        other._fd = -1;
    }
    return *this;
}

bool UARTDev::setConfig(const UARTConfig& config){
    if (_fd < 0) {
        _last_error = "Device not open";
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if(tcgetattr(_fd, &tty) != 0){
        _last_error = "Failed to get terminal attributes: " + std::string(std::strerror(errno));
        return false;
    }

    cfsetospeed(&tty, getBaudRate(config.baud_rate));
    cfsetispeed(&tty, getBaudRate(config.baud_rate));

    tty.c_cflag &= ~CSIZE;
    switch(config.data_bits){
        case DataBits::FIVE:
            tty.c_cflag |= CS5;
            break;
        case DataBits::SIX:
            tty.c_cflag |= CS6;
            break;
        case DataBits::SEVEN:
            tty.c_cflag |= CS7;
            break;
        case DataBits::EIGHT:
            tty.c_cflag |= CS8;
            break;
        default:
            _last_error = "Invalid data bits configuration";
            return false;
    }

    switch(config.parity){
        case Parity::NONE:
            tty.c_cflag &= ~PARENB;
            break;
        case Parity::EVEN:
            tty.c_cflag |= PARENB;
            tty.c_cflag &= ~PARODD;
            break;
        case Parity::ODD:
            tty.c_cflag |= PARENB | PARODD;
            break;
        default:
            _last_error = "Invalid parity configuration";
            return false;
    }

    switch(config.stop_bits){
        case StopBits::ONE:
            tty.c_cflag &= ~CSTOPB;
            break;
        case StopBits::TWO:
            tty.c_cflag |= CSTOPB;
            break;
        default:
            _last_error = "Invalid stop bits configuration";
            return false;
    }

    switch(config.flow_control){
        case FlowControl::NONE:
            tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
            tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
            break;
        case FlowControl::RTS_CTS:
            tty.c_cflag |= CRTSCTS; // Enable hardware flow control
            tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
            break;
        case FlowControl::XON_XOFF:
            tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
            tty.c_iflag |= IXON | IXOFF | IXANY; // Enable software flow control
            break;
        default:
            _last_error = "Invalid flow control configuration";
            return false;
    }

    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG); // Raw input mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of bytes

    tty.c_oflag &= ~OPOST; // Raw output mode
    tty.c_oflag &= ~ONLCR; // Don't convert newline to carriage return + newline

    tty.c_cc[VTIME] = config.read_timeout_ms.count() / 100; // VTIME is in deciseconds
    tty.c_cc[VMIN] = config.blocking ? 1 : 0;

    if(tcsetattr(_fd, TCSANOW, &tty) != 0){
        _last_error = "Failed to set terminal attributes: " + std::string(std::strerror(errno));
        return false;
    }

    _config = config;
    return true;
}

int UARTDev::getBaudRate(BaudRate baud_rate) {
    switch(baud_rate){
        case BaudRate::BAUD_9600: return B9600;
        case BaudRate::BAUD_19200: return B19200;
        case BaudRate::BAUD_38400: return B38400;
        case BaudRate::BAUD_57600: return B57600;
        case BaudRate::BAUD_115200: return B115200;
        case BaudRate::BAUD_230400: return B230400;
        case BaudRate::BAUD_460800: return B460800;
        default: return B115200; // Default to 115200 if invalid
    }
} 

size_t UARTDev::read(uint8_t* data, size_t size){
    if(_fd < 0) {
        _last_error = "Device not open";
        return 0;
    }
    std::lock_guard<std::mutex> lock(_mutex);

    auto timeout = std::chrono::milliseconds(_config.read_timeout_ms);

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(_fd, &read_fds);

    struct timeval tv;
    tv.tv_sec = timeout.count() / 1000;
    tv.tv_usec = (timeout.count() % 1000) * 1000;

    int ret = select(_fd + 1, &read_fds, nullptr, nullptr, &tv);
    if(ret < 0){
        _last_error = "Failed to wait for data: " + std::string(std::strerror(errno));
        return 0;
    }else if(ret == 0){
        _last_error = "Read timeout";
        return 0;
    }

    ssize_t r_bytes = ::read(_fd, data, size);
    if(r_bytes < 0){
        _last_error = "Failed to read from UART: " + std::string(std::strerror(errno));
        return 0;
    }
    return static_cast<size_t>(r_bytes);
}

bool UARTDev::readExact(uint8_t* data, size_t size){    
    size_t total_read = 0;
    while(total_read < size){
        size_t r_bytes = read(data + total_read, size - total_read);
        if(r_bytes == 0){
            return false;   // Timeout or error
        }
        total_read += r_bytes;
    }
    return total_read == size;
}

std::vector<uint8_t> UARTDev::readUntil(uint8_t delimiter, size_t max_size){
    std::vector<uint8_t> buffer;
    buffer.reserve(max_size);

    while(buffer.size() < max_size){
        uint8_t byte;
        size_t r_bytes = read(&byte, 1);

        if(r_bytes == 0){
            break; // Timeout or error
        }

        buffer.push_back(byte);
        if(byte == delimiter){
            break; // Delimiter found
        }
    }
    return buffer;
}

std::string UARTDev::readLine(size_t max_size){
    auto buffer = readUntil('\n', max_size);
    if(!buffer.empty()){
        return "";
    }

    while (!buffer.empty() && (buffer.back() == '\n' || buffer.back() == '\r')) {
        buffer.pop_back();
    }
    return std::string(buffer.begin(), buffer.end());
}

size_t UARTDev::write(const uint8_t* data, size_t size){
    if(_fd < 0) {
        _last_error = "Device not open";
        return 0;
    }
    std::lock_guard<std::mutex> lock(_mutex);

    ssize_t w_bytes = ::write(_fd, data, size);
    if(w_bytes < 0){
        _last_error = "Failed to write to UART: " + std::string(std::strerror(errno));
        return 0;
    }
    return static_cast<size_t>(w_bytes);
}

size_t UARTDev::write(const std::string& data){
    return write(reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

size_t UARTDev::available() {
    if(_fd < 0) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(_mutex);

    int bytes;
    if(ioctl(_fd, FIONREAD, &bytes) < 0){
        _last_error = "Failed to get available bytes: " + std::string(std::strerror(errno));

        return 0;
    }
    return static_cast<size_t>(bytes);
}

void UARTDev::flushInput(){
    if(_fd >= 0){
        std::lock_guard<std::mutex> lock(_mutex);
        tcflush(_fd, TCIFLUSH);
    }
}

void UARTDev::flushOutput(){
    if(_fd >= 0){
        std::lock_guard<std::mutex> lock(_mutex);
        tcflush(_fd, TCOFLUSH);
    }
}

void UARTDev::flush(){
    if(_fd >= 0){
        std::lock_guard<std::mutex> lock(_mutex);
        tcflush(_fd, TCIOFLUSH);
    }
}

bool UARTDev::setBaudRate(BaudRate baud_rate){
    std::lock_guard<std::mutex> lock(_mutex);

    if(_fd < 0) {
        _last_error = "Device not open";
        return false;
    }

    struct termios tty;
    if(tcgetattr(_fd, &tty) != 0){
        _last_error = "Failed to get terminal attributes: " + std::string(std::strerror(errno));
        return false;
    }

    cfsetospeed(&tty, getBaudRate(baud_rate));
    cfsetispeed(&tty, getBaudRate(baud_rate));

    if(tcsetattr(_fd, TCSANOW, &tty) != 0){
        _last_error = "Failed to set terminal attributes: " + std::string(std::strerror(errno));
        return false;
    }
    _config.baud_rate = baud_rate;
    return true;
}

void UARTDev::setTimeout(std::chrono::milliseconds timeout){
    std::lock_guard<std::mutex> lock(_mutex);

    _config.read_timeout_ms = timeout;
    if(_fd >= 0) {
        struct termios tty;
        if(tcgetattr(_fd, &tty) == 0){
            tty.c_cc[VTIME] = timeout.count() / 100; // VTIME is in deciseconds
            tcsetattr(_fd, TCSANOW, &tty);
        }
    }
}

} // namespace hal
} // namespace robotics



