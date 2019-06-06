// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatSDO.h"
#include "ros/ros.h"

extern "C" {
#include "ethercat.h"
}

namespace march4cpp {
// TODO(Isha, Martijn, Tim) refactor this with more generic types
int sdo_bit8(int slave, uint32_t index, uint8_t sub, uint8_t value) {
  ROS_INFO("sdo_bit8: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X", slave,
           index, sub, value);
  bool success =
      ec_SDOwrite(slave, index, sub, FALSE, 1, &value, EC_TIMEOUTRXM);
  if (!success) {
    ROS_WARN("SDO write of value 0x%X to slave with index %d, to register "
             "0x%X, subindex %d failed",
             value, slave, index, sub);
  }
  return success;
}

int sdo_bit16(int slave, uint32_t index, uint8_t sub, uint16_t value) {
  ROS_INFO("sdo_bit16: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X", slave,
           index, sub, value);
  bool success =
      ec_SDOwrite(slave, index, sub, FALSE, 2, &value, EC_TIMEOUTRXM);
  if (!success) {
    ROS_WARN("SDO write of value 0x%X to slave with index %d, to register "
             "0x%X, subindex %d failed",
             value, slave, index, sub);
  }
  return success;
}

int sdo_bit32(int slave, uint32_t index, uint8_t sub, uint32_t value) {
  ROS_INFO("sdo_bit32: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X", slave,
           index, sub, value);
  bool success =
      ec_SDOwrite(slave, index, sub, FALSE, 4, &value, EC_TIMEOUTRXM);
  if (!success) {
    ROS_WARN("SDO write of value 0x%X to slave with index %d, to register "
             "0x%X, subindex %d failed",
             value, slave, index, sub);
  }
  return success;
}

} // namespace march4cpp
