/**
  ******************************************************************************
  * @file    I2CDev.c
  * @author  UrbanIzzy
  * @date    Jan 20, 2026
  * @brief   This file contain class header of MPU9250/MPU9255 driver
  ******************************************************************************
*/
#ifndef _I2C_DEV_H_
#define _I2C_DEV_H_ 

/* Includes ------------------------------------------------------------------*/
#include <string>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* Classes -------------------------------------------------------------------*/
class I2CDev {
    protected:
        int m_fd;
        uint8_t m_addr;
        std::string m_bus;

    public:
        I2CDev(const std::string& bus, uint8_t addr)
            : m_fd(-1), m_addr(addr), m_bus(bus) {};
        virtual ~I2CDev() {if (m_fd >= 0) close(m_fd);};

        bool open_device(){
            m_fd =open(m_bus.c_str(), O_RDWR);
            if(m_fd < 0) return false;
            if(ioctl(m_fd, I2C_SLAVE, m_addr) < 0) return false;
            return true;
        }

        bool write_reg(uint8_t reg, uint8_t data){
            uint8_t buf[2] = {reg, data};
            return write(m_fd, buf, 2) == 2;
        }

        bool read_reg(uint8_t reg, uint8_t &data){
            if(write(m_fd, &reg, 1) != 1) return false;
            return read(m_fd, &data, 1) == 1;
        }

        bool read_burst(uint8_t s_reg, uint8_t *data, uint8_t len){
            if(write(m_fd, &s_reg, 1) != 1) return false;
            return read(m_fd, data, len) == len;
        }
};
#endif