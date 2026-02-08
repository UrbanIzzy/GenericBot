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
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdarg>
#include <iostream>

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

class Logger {
    public:
        static void init(
            const std::string& logFilePath = "",
            LogLevel consolLevel = LogLevel::INFO,
            LogLevel fileLevel = LogLevel::DEBUG) {
                auto& instance = getInstance();
                std::lock_guard<std::mutex> lock(instance._mutex);
                
                instance._consoleLevel = consolLevel;
                instance._fileLevel = fileLevel;
                if(!logFilePath.empty() && !instance._logFile.is_open()) {
                    instance._logFile.open(logFilePath, std::ios::app);
                    if(!instance._logFile.is_open()){
                        std::cerr << "Logger: Failed to open" << logFilePath << std::endl;
                    }
                }
        }
        
        static void shutdown(){
            auto& instance = getInstance();
            std::lock_guard<std::mutex> lock(instance._mutex);
            
            if(instance._logFile.is_open())
                instance._logFile.close();
        }

        static void log(LogLevel level, const char* module,  const char* format, ...){
            auto& instance = getInstance();

            if(level < instance._consoleLevel && level < instance._fileLevel){
                return;
            }
            char buffer[2048];
            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
        
            instance.logMessage(level, module, buffer);
        }

        static Logger& getInstance(){
            static Logger instance;
            return instance;
        }

        void setConsoleLevel(LogLevel level){
            std::lock_guard<std::mutex> lock(_mutex);
            _consoleLevel = level;
        }

        void setFileLevel(LogLevel level){
            std::lock_guard<std::mutex> lock(_mutex);
            _fileLevel = level;
        }

    private:
        LogLevel _consoleLevel;
        LogLevel _fileLevel;
        std::ofstream _logFile;
        std::mutex _mutex;

        Logger()
            : _consoleLevel(LogLevel::INFO)
            , _fileLevel(LogLevel::DEBUG) {}

        ~Logger() {
            if(_logFile.is_open()){
                _logFile.close();
            }
        }

        Logger(const Logger&) = delete;
        Logger& operator=(const Logger&) = delete;

        std::string formatTimestamp() const {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

            std::tm tm;
            localtime_r(&time_t, &tm);

            std::ostringstream oss;
            oss << std::put_time(&tm , "%Y-%m-%d %H:%M:%S") << "," << std::setfill('0') << std::setw(6) << us.count();

            return oss.str();
        }

        const char* getLevelString(LogLevel level) const {
            switch(level){
                    case LogLevel::DEBUG: return "DEBUG";
                    case LogLevel::INFO:  return "-INFO";
                    case LogLevel::WARN:  return "WARNN";
                    case LogLevel::ERROR: return "ERROR";
                    case LogLevel::FATAL: return "FATAL";
                    default: return "UNKNOWN";
            }
        }

        const char* getLevelColor(LogLevel level) const {
            switch(level){
                    case LogLevel::DEBUG: return Color::CYAN;
                    case LogLevel::INFO: return Color::GREEN;
                    case LogLevel::WARN: return Color::YELLOW;
                    case LogLevel::ERROR: return Color::RED;
                    case LogLevel::FATAL: return Color::MAGENTA;
                    default: return Color::RESET;
            }
        }

        void logMessage(LogLevel level, const char* module, const char* message){
            std::lock_guard<std::mutex> lock(_mutex);

            std::ostringstream oss;
            oss << formatTimestamp() << " " 
                << "[" << getLevelString(level) << "]"
                << "[" << std::setw(15) << std::left << module << "]"
                << message;

            std::string formatted = oss.str();
            
            if(level >= _fileLevel){
                std::cout << getLevelColor(level) << formatted << Color::RESET << std::endl;
            }

            if(level >= _fileLevel && _logFile.is_open()) {
                _logFile << formatted << std::endl;
                _logFile.flush();
            }
        }
};

#define LOG_DEBUG(module, ...) \
    robotics::Logger::log(robotics::LogLevel::DEBUG, module, __VA_ARGS__)

#define LOG_INFO(module, ...) \
    robotics::Logger::log(robotics::LogLevel::INFO, module, __VA_ARGS__)

#define LOG_WARN(module, ...) \
    robotics::Logger::log(robotics::LogLevel::WARN, module, __VA_ARGS__)

#define LOG_ERROR(module, ...) \
    robotics::Logger::log(robotics::LogLevel::ERROR, module, __VA_ARGS__)

#define LOG_FATAL(module, ...) \
    robotics::Logger::log(robotics::LogLevel::FATAL, module, __VA_ARGS__)

} // namespace robotics