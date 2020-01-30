// Copyright 2019 Project March.

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/EncoderIncremental.h>
#include <cmath>
#include <ros/ros.h>

namespace march
{
EncoderIncremental::EncoderIncremental(int numberOfBits) : Encoder(numberOfBits)
{
}

float EncoderIncremental::getAngleRad(uint8_t ActualPositionByteOffset)
{
  return IUtoRad(getAngleIU(ActualPositionByteOffset));
}

int EncoderIncremental::RadtoIU(float rad)
{
  return static_cast<int>(rad * totalPositions / (2 * M_PI));
}

float EncoderIncremental::IUtoRad(int iu)
{
  return static_cast<float>(iu) * 2 * M_PI / totalPositions;
}
} //  namespace march
