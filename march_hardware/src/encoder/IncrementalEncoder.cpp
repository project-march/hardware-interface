// Copyright 2019 Project March.
#include "march_hardware/encoder/IncrementalEncoder.h"

#include <cmath>

namespace march
{
IncrementalEncoder::IncrementalEncoder(size_t number_of_bits) : Encoder(number_of_bits)
{
}

double IncrementalEncoder::getAngleRad(uint8_t byte_offset)
{
  return this->toRad(Encoder::getAngleIU(byte_offset));
}

double IncrementalEncoder::toRad(int32_t iu)
{
  return iu * 2 * M_PI / Encoder::getTotalPositions();
}
}  //  namespace march
