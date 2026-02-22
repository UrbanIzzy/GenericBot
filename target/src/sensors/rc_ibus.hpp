/**
  ******************************************************************************
  * @file    rc_ibus.hpp
  * @author  UrbanIzzy
  * @date    Feb 18, 2026
  * @brief   FlySky IBUS protocol receiver implementation
  ******************************************************************************
*/

#pragma once

#include "rc_base.hpp"
#include <cstdint>

namespace robotics {
namespace sensors {

class IBUS_Reciver : public RCBase {
    public:
      explicit IBUS_Reciver(const std::string& dev);
      ~IBUS_Reciver() override = default;
    
      bool initialize() override;
      bool readChannels(RCData& data) override;
      int getChannelNum() const override { return RCDefs::CHANNEL_NUM; }
      std::string getProtocolName() const override { return "IBUS"; }

    private:
      bool sync();
      bool readPacket(uint8_t* buffer);
      bool parsePacket(const uint8_t* packet, RCData& data);
      bool validatePacket(const uint8_t* packet) const;
      uint16_t calcIBUSChecksum(const uint8_t* data, size_t len) const;
      int16_t extractChannelValue(const uint8_t* data, int index) const;

      bool _is_synced;
      uint8_t _packet_buffer[RCDefs::PACKET_SIZE];
      const std::string _name = "IBUS Receiver";
};

}
}