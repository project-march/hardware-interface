// Copyright 2019 Project March.
#include "march_hardware/EncoderIncremental.h"

#include <cmath>

namespace march
{
EncoderIncremental::EncoderIncremental(size_t number_of_bits) : Encoder(number_of_bits)
{
}

double EncoderIncremental::getAngleRad(uint8_t byte_offset)
{
  return this->toRad(Encoder::getAngleIU(byte_offset));
}

double EncoderIncremental::toRad(int32_t iu)
{
  return iu * 2 * M_PI / Encoder::getTotalPositions();
}
}  //  namespace march
