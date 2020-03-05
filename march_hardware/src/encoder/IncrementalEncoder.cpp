// Copyright 2019 Project March.
#include "march_hardware/encoder/IncrementalEncoder.h"

#include <cmath>

namespace march
{
IncrementalEncoder::IncrementalEncoder(size_t number_of_bits, double transmission)
  : Encoder(number_of_bits), transmission_(transmission)
{
}

double IncrementalEncoder::getAngleRad(uint8_t byte_offset)
{
  return this->toRad(Encoder::getAngleIU(byte_offset));
}

double IncrementalEncoder::toRad(int32_t iu)
{
  return iu * this->transmission_ * 2 * M_PI / Encoder::getTotalPositions();
}
}  //  namespace march
