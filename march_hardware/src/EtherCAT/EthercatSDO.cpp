// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatSDO.h"
#include "ros/ros.h"

extern "C"
{
#include "ethercat.h"
}

namespace march4cpp
{
bool sendSDOMessage(int slave, uint32_t index, uint8_t sub, SDOMessageLength length, uint32_t value)
{
  ROS_DEBUG("SDO message: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X", slave, index, sub, value);
  uint8_t value8bit = static_cast<uint8_t>(value);
  uint16_t value16bit = static_cast<uint16_t>(value);
  uint32_t value32bit = static_cast<uint32_t>(value);
  switch (length)
  {
    case SDOMessageLength::bit8 :
      return ec_SDOwrite(slave, index, sub, FALSE, length, &value8bit, EC_TIMEOUTRXM);
    case SDOMessageLength::bit16 :
      return ec_SDOwrite(slave, index, sub, FALSE, length, &value16bit, EC_TIMEOUTRXM);
    case SDOMessageLength::bit32 :
      return ec_SDOwrite(slave, index, sub, FALSE, length, &value32bit, EC_TIMEOUTRXM);
    default :
      ROS_WARN("SDO message length not defined. Message not sent");
      return false;
  }
}

}  // namespace march4cpp
