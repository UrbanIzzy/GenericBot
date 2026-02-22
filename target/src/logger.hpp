/**
  ******************************************************************************
  * @file    logger.hpp
  * @author  UrbanIzzy
  * @date    Feb 3, 2026
  * @brief   Lightweight thread-safe logger
  ******************************************************************************
*/


#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <chrono>
#include <thread>
#include <queue>
#include <condition_variable>
#include <cstdarg>
#include <atomic>

namespace robotics {

enum class LogLevel {
    DEBUG = 0,
    INFO  = 1,
    WARN  = 2,
    ERROR = 3,
    FATAL = 4 
};

namespace Color {
    constexpr const char* RESET = "\033[0m";
    constexpr const char* CYAN = "\033[36m";
    constexpr const char* GREEN = "\033[32m";
    constexpr const char* YELLOW = "\033[33m";
    constexpr const char* RED = "\033[31m";
    constexpr const char* MAGENTA = "\033[1;35m";
}

struct LogMessage {
    uint64_t timestamp_us;
    LogLevel level;
    std::string module;
    std::string message;
    
    size_t estimateSize() const {
        return sizeof(timestamp_us) + sizeof(level) + module.size() + message.size() + 32; // 32 bytes overhead
    }
};

class Logger {
    public:
        static void init(
            const std::string& file_name,
            LogLevel console_level = LogLevel::INFO,
            LogLevel file_level = LogLevel::DEBUG,
            size_t queue_capacity = 2024); // Default 2MB queue capacity

        static void shutdown();

        static void log(LogLevel level, const char* module, const char* fmt, ...);

        struct Statistics {
            uint64_t total_messages;
            uint64_t dropped_messages;
            size_t queue_size;
            size_t queue_capacity;
            size_t current_queue_memory;
        };

        static Statistics getStatistics();

    private:
        Logger() = default;
        ~Logger() = default;

        static Logger& getInstance();
        void threadFunc();
        static std::string extructModuleName(const char* module);
        static std::string formatTimestamp(uint64_t timestamp_us);
        static const char* getLevelString(LogLevel level);
        static const char* getLevelColor(LogLevel level);

        void writeMessage(const LogMessage& msg);

        std::string _file_name;
        LogLevel _consoleLevel;
        LogLevel _fileLevel;
        size_t _queue_capacity;

        std::queue<LogMessage> _msg_queue;
        std::mutex _queue_mutex;
        std::condition_variable _queue_cv;
        size_t _current_queue_memory;
        
        std::unique_ptr<std::thread> _logger_thread;
        std::atomic<bool> _running;

        std::ofstream _log_file;

        std::atomic<uint64_t> _total_messages;
        std::atomic<uint64_t> _dropped_messages;

        static std::atomic<bool> _initialized;
};



#define LOG_DEBUG(module, ...) \
    robotics::Logger::log(robotics::LogLevel::DEBUG, __PRETTY_FUNCTION__, module, ##__VA_ARGS__)

#define LOG_INFO(module, ...) \
    robotics::Logger::log(robotics::LogLevel::INFO, __PRETTY_FUNCTION__, module, ##__VA_ARGS__)

#define LOG_WARN(module, ...) \
    robotics::Logger::log(robotics::LogLevel::WARN, __PRETTY_FUNCTION__, module, ##__VA_ARGS__)

#define LOG_ERROR(module, ...) \
    robotics::Logger::log(robotics::LogLevel::ERROR, __PRETTY_FUNCTION__, module, ##__VA_ARGS__)

#define LOG_FATAL(module, ...) \
    robotics::Logger::log(robotics::LogLevel::FATAL, __PRETTY_FUNCTION__, module, ##__VA_ARGS__)

}