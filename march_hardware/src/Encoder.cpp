// Copyright 2019 Project March.

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Encoder.h>
#include <cmath>
#include <ros/ros.h>

namespace march
{
Encoder::Encoder(int numberOfBits)
{
  ROS_ASSERT_MSG(numberOfBits > 0 && numberOfBits <= 32, "Encoder resolution of %d is not within range (0, 32)",
                 numberOfBits);
  this->totalPositions = static_cast<int>(pow(2, numberOfBits) - 1);
  this->slaveIndex = -1;
}

int Encoder::getAngleIU(uint8_t ActualPositionByteOffset)
{
  if (this->slaveIndex == -1)
  {
    ROS_FATAL("Encoder has slaveIndex of -1");
  }
  union bit32 return_byte = get_input_bit32(this->slaveIndex, ActualPositionByteOffset);
  ROS_DEBUG("Encoder read (IU): %d", return_byte.i);
  return return_byte.i;
}

void Encoder::setSlaveIndex(int slaveIndex)
{
  this->slaveIndex = slaveIndex;
}

int Encoder::getSlaveIndex() const
{
  return this->slaveIndex;
}
}  // namespace march
