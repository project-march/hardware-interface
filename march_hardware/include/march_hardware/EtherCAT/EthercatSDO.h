// Copyright 2019 Project March.
#ifndef MARCH4CPP__SDO_H
#define MARCH4CPP__SDO_H

#include <stdint.h>

namespace march4cpp
{

enum class SDOMessageLength
{
  bit8,
  bit16,
  bit32
};

bool sendSDOMessage(int slave, uint32_t index, uint8_t sub, SDOMessageLength length, uint32_t value);

}  // namespace march4cpp
#endif  // MARCH4CPP__SDO_H
