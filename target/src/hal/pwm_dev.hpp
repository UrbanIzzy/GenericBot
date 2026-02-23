/**
  ******************************************************************************
  * @file    pwm_dev.hpp
  * @author  UrbanIzzy
  * @date    Feb 22, 2026
  * @brief   PWM Hardware Abstraction Layer
  ******************************************************************************
*/
#pragma once

#include <string>
#include <cstdint>

namespace robotics {
namespace hal { 

struct PWMConfig {
    uint32_t period_ns; 
    uint32_t duty_ns; 
    bool polarity_inverted; 
    bool enabled;

    float getFrequencyHz() const {
        if(period_ns == 0) return 0.0f;
        return 1e9f / period_ns;
    }

    float getDutyCycle() const {
        if(period_ns == 0) return 0.0f;
        return (duty_ns) * 100 / (period_ns);
    }
};

namespace PWMFrequency {
    constexpr uint32_t FREQ_50HZ = 20000000; // 20ms period
    constexpr uint32_t FREQ_100HZ = 10000000; // 10ms period
    constexpr uint32_t FREQ_400HZ = 2500000;  // 2.5ms period
    constexpr uint32_t FREQ_1KHZ = 1000000; // 1ms period
    constexpr uint32_t FREQ_2KHZ = 500000;  // 0.5ms period
    constexpr uint32_t FREQ_20KHZ = 50000;  // 0.5ms period
}

namespace PWMPulseWidth {
    constexpr uint32_t MIN_US = 1000;       // 1ms pulse width
    constexpr uint32_t MAX_US = 2000;       // 2ms pulse width
    constexpr uint32_t NEUTRAL_US = 1500;   // 1.5ms pulse width

    constexpr uint32_t MIN_NS = 1000000;    // 1ms pulse width
    constexpr uint32_t MAX_NS = 2000000;    // 2ms pulse width
    constexpr uint32_t NEUTRAL_NS = 1500000; // 1.5ms pulse width
}

class PWMDev {
    public:
        virtual ~PWMDev() = default;

        virtual bool initialize() = 0;
        virtual void deinit() = 0;

        virtual bool setPeriod(uint32_t period_ns) = 0;   
        virtual bool setDutyCycle(uint32_t duty_ns) = 0; // duty_cycle: 0.0 - 100.0
        virtual bool setDutyCyclePercent(float duty_precent) = 0; // duty_cycle: 0.0 - 100.0
        virtual bool setDutyCycleMicro(uint32_t duty_us) = 0; // duty_cycle: 0.0 - 100.0
        virtual bool setPolarity(bool inverted) = 0;

        virtual bool enable() = 0;
        virtual bool disable() = 0;
        virtual bool isEnabled() const = 0;

        virtual PWMConfig getConfig() const = 0;
        virtual std::string getLastError() const = 0;
        virtual int getChipNum() const = 0;
        virtual int getChannelNum() const = 0;

        virtual bool setFrequency(float frequency_hz) {
            if(frequency_hz <= 0.0f) return false;
            uint32_t period_ns = static_cast<uint32_t>(1e9f / frequency_hz);
            return setPeriod(period_ns);
        }
    
    protected:
        std::string _last_error;
};

}
}