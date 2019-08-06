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
  int byteLength;
  uint8_t value8bit;
  uint16_t value16bit;
  uint32_t value32bit;
  switch (length){
    case SDOMessageLength::bit8 :
      byteLength = 1;
      value8bit = static_cast<uint8_t>(value);
      return ec_SDOwrite(slave, index, sub, FALSE, byteLength, &value8bit, EC_TIMEOUTRXM);
    case SDOMessageLength::bit16 :
      byteLength = 2;
      value16bit = static_cast<uint16_t>(value);
      return ec_SDOwrite(slave, index, sub, FALSE, byteLength, &value16bit, EC_TIMEOUTRXM);
    case SDOMessageLength::bit32 :
      byteLength = 4;
      value32bit = static_cast<uint32_t>(value);
      return ec_SDOwrite(slave, index, sub, FALSE, byteLength, &value32bit, EC_TIMEOUTRXM);
  }
}

}  // namespace march4cpp
