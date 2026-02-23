/**
  ******************************************************************************
  * @file    pwm_dev_linux.hpp
  * @author  UrbanIzzy
  * @date    Feb 22, 2026
  * @brief   Linux sysfs PWM implementation
  ******************************************************************************
*/
#pragma once

#include "pwm_dev.hpp"
#include <fstream>
#include <mutex>

namespace robotics {
namespace hal {


class LinuxPWMDev : public PWMDev {
    public:
        LinuxPWMDev(int chip_num, int channel_num);
        ~LinuxPWMDev() override;

        bool initialize() override;
        void deinit() override;

        bool setPeriod(uint32_t period_ns) override;
        bool setDutyCycle(uint32_t duty_ns) override;
        bool setDutyCyclePercent(float duty_precent) override;
        bool setDutyCycleMicro(uint32_t duty_us) override;
        bool setPolarity(bool inverted) override;

        bool enable() override;
        bool disable() override;
        bool isEnabled() const override;

        PWMConfig getConfig() const override;
        std::string getLastError() const override;

        int getChipNum() const override { return _chip_num; }
        int getChannelNum() const override { return _channel_num; }


    private:
        bool exportChannel();
        void unexportChannel();
        bool isExported() const;
        bool writeFile(const std::string& path, const std::string& value);
        bool readFile(const std::string& path, std::string& value); //\tdb: was const function
        std::string getPWMPath() const;
        std::string getChipPath() const;

        int _chip_num;
        int _channel_num;
        
        PWMConfig _config;
        
        bool _exported;
        bool _initialized;

        mutable std::mutex _mutex;
};




}}