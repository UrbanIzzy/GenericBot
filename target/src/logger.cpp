/**
 * @file logger.cpp
 * @brief Async logger implementation
 */

#include "logg.hpp"
#include <iostream>
#include <cstring>
#include <ctime>
#include <sys/time.h>
#include <sstream>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

namespace robotics {

std::atomic<bool> Logger::_initialized{false};

void Logger::init(const std::string& file_name, LogLevel consol_level, 
                  LogLevel file_level, size_t queue_capacity) {
    auto& instance = getInstance();
    // std::lock_guard<std::mutex> lock(instance._mutex);

    if(_initialized.exchange(true)){
        std::cerr << "Logger is already initialized" << std::endl;
        return;
    }

    instance._file_name = file_name;
    instance._consoleLevel = consol_level;
    instance._fileLevel = file_level;
    instance._queue_capacity = queue_capacity * 1024;
    instance._current_queue_memory = 0;
    instance._total_messages = 0;
    instance._dropped_messages = 0;

    instance._log_file.open(file_name, std::ios::out | std::ios::app);
    if(!instance._log_file.is_open()){
        std::cerr << "Logger: Failed to open log file: " << file_name << std::endl;
        _initialized = false;
        return; 
    }    

    instance._running = true;
    instance._logger_thread = std::make_unique<std::thread>(&Logger::threadFunc, &instance);
#ifdef __linux__
    // Set thread to real-time priority
    struct sched_param param;
    param.sched_priority = 10;
    if(pthread_setschedparam(instance._logger_thread->native_handle(), SCHED_FIFO, &param) == 0){
        std::cout << "Logger thread set to real-time priority" << std::endl;
    } else {
        std::cerr << "Failed to set logger thread priority: " << strerror(errno) << std::endl;
    }
#endif
    std::cout << "Logger initialized. Log file: " << file_name << std::endl;
}

void Logger::shutdown() {
    auto& instance = getInstance();
    if(!_initialized.exchange(false)){
        std::cerr << "Logger is not initialized" << std::endl;
        return;
    }

    instance._running = false;
    instance._queue_cv.notify_all();
    if(instance._logger_thread && instance._logger_thread->joinable()){
        instance._logger_thread->join();
    }

    if(instance._log_file.is_open()){
        instance._log_file.close();
    }

    _initialized = false;

    std::cout << "Logger shutdown complete" << std::endl;
}

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}


void Logger::log(LogLevel level, const char* module, const char* fmt, ...){
    if(!_initialized){
        return;
    }

    auto& instance = getInstance();

    if(level < instance._consoleLevel && level < instance._fileLevel){
        return;
    }

    char buffer[2048];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    LogMessage msg;
    msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    msg.level = level;
    msg.module = extructModuleName(module);
    msg.message = buffer;

    size_t msg_size = msg.estimateSize();

    {
        std::lock_guard<std::mutex> lock(instance._queue_mutex);

        while(instance._current_queue_memory + msg_size > instance._queue_capacity &&
             !instance._msg_queue.empty()){
            const auto& oldest = instance._msg_queue.front();
            instance._current_queue_memory -= oldest.estimateSize();
            instance._msg_queue.pop();
            instance._dropped_messages++;
        }
        instance._msg_queue.push(std::move(msg));
        instance._current_queue_memory += msg_size;
    }
    instance._queue_cv.notify_one();
}

void Logger::threadFunc() {
    while(_running) {
        LogMessage msg;
        {
            std::unique_lock<std::mutex> lock(_queue_mutex);

            _queue_cv.wait(lock, [this](){
                return !_msg_queue.empty() || !_running.load();
            });

            if(!_running && _msg_queue.empty()){
                break;
            }

            if(_msg_queue.empty()){
                continue;
            }
            
            msg = std::move(_msg_queue.front());
            _msg_queue.pop();
            _current_queue_memory -= msg.estimateSize();
            
        }

        writeMessage(msg);
        _total_messages++;
    }

    while(true) { // Flush remaining messages
        LogMessage msg;

        {
            std::unique_lock<std::mutex> lock(_queue_mutex);

            if(_msg_queue.empty()){
                break;
            }

            msg = std::move(_msg_queue.front());
            _msg_queue.pop();
        }

        writeMessage(msg);
        _total_messages++;
    }
}

void Logger::writeMessage(const LogMessage& msg) {
    std::string timstamp_srt = formatTimestamp(msg.timestamp_us);
    std::string level_str = getLevelString(msg.level);

    std::ostringstream oss;
    oss << "[" << timstamp_srt << "] "
        << "[" << level_str << "] "
        << "[" << msg.module << "] "
        << msg.message;

    std::string formatted = oss.str();

    if(msg.level >= _consoleLevel){
        const char* color = getLevelColor(msg.level);
        std::cout << color << formatted << Color::RESET << std::endl;
    }

    if(msg.level >= _fileLevel && _log_file.is_open()){
        _log_file << formatted << std::endl;
        _log_file.flush();
    }
}

std::string Logger::extructModuleName(const char* module) {
    std::string mod_str(module);

    size_t paren_pos = mod_str.find('(');
    if(paren_pos != std::string::npos){
        return mod_str;
    }

    std::string before_paren = mod_str.substr(0, paren_pos);

    size_t last_colon = before_paren.find_last_of("::");
    if(last_colon != std::string::npos){
        size_t last_space = before_paren.rfind(" ");
        if(last_space != std::string::npos){
            return before_paren.substr(last_space + 1);
        }
        return before_paren;
    }

    size_t second_last_colon = before_paren.rfind("::", last_colon - 1);
    if(second_last_colon != std::string::npos){
        size_t last_space = before_paren.rfind(" ");
        if(last_space != std::string::npos){
            return before_paren.substr(second_last_colon + 1);
        }
        return before_paren;
    }
    return before_paren.substr(second_last_colon + 2);
}

std::string Logger::formatTimestamp(uint64_t timestamp_us){
    std::time_t time_sec = timestamp_us / 1000000;
    uint64_t microseconds = timestamp_us % 1000000;

    std::tm tm_time;
    localtime_r(&time_sec, &tm_time);

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d.%06lu",
                tm_time.tm_year + 1900, 
                tm_time.tm_mon + 1, 
                tm_time.tm_mday,
                tm_time.tm_hour, 
                tm_time.tm_min, 
                tm_time.tm_sec, 
                microseconds);

    return buffer;
}

const char* Logger::getLevelString(LogLevel level) {
    switch(level){
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

const char* Logger::getLevelColor(LogLevel level) {
    switch(level){
        case LogLevel::DEBUG: return Color::CYAN;
        case LogLevel::INFO:  return Color::GREEN;
        case LogLevel::WARN:  return Color::YELLOW;
        case LogLevel::ERROR: return Color::RED;
        case LogLevel::FATAL: return Color::MAGENTA;
        default: return Color::RESET;
    }
}

Logger::Statistics Logger::getStatistics() {
    auto& instance = getInstance();

    Statistics stats;
    {
        std::lock_guard<std::mutex> lock(instance._queue_mutex);
        stats.queue_size = instance._msg_queue.size();
        stats.current_queue_memory = instance._current_queue_memory;

    }

    stats.total_messages = instance._total_messages;
    stats.dropped_messages = instance._dropped_messages;
    stats.queue_capacity = instance._queue_capacity;
    return stats;
}

}