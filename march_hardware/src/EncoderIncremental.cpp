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
} //  namespace march
