// Copyright 2019 Project March.
#ifndef MARCH4CPP__SDO_H
#define MARCH4CPP__SDO_H

#include <stdint.h>

namespace march4cpp
{

enum SDOMessageLength
{
  // Give enum value of amount of bytes
  bit_8 = 1,
  bit_16 = 2,
  bit_32 = 4
};

bool sendSDOMessage(int slave, uint32_t index, uint8_t sub, SDOMessageLength length, uint32_t value);

}  // namespace march4cpp
#endif  // MARCH4CPP__SDO_H
