// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_INCREMENTAL_H
#define MARCH_HARDWARE_ENCODER_INCREMENTAL_H

#include <ostream>
#include "Encoder.h"

namespace march{
class EncoderIncremental : public Encoder
{
private:

public:
  EncoderIncremental()
  {
  }

  float getAngleRad(uint8_t ActualPositionByteOffset);

  float IUtoRad(int iu);
  int RadtoIU(float rad);

  EncoderIncremental(int numberOfBits);

  /** @brief Override comparison operator */
  friend bool operator==(const EncoderIncremental& lhs, const EncoderIncremental& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.totalPositions == rhs.totalPositions;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const EncoderIncremental& encoderIncremental)
  {
    return os << "slaveIndex: " << encoderIncremental.slaveIndex << ", "
              << "totalPositions: " << encoderIncremental.totalPositions;
  }
};

} // namespace march
#endif  //  MARCH_HARDWARE_ENCODER_INCREMENTAL_H