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
  EncoderIncremental(int numberOfBits);

  /** @brief Override comparison operator */
  friend bool operator==(const Encoder& lhs, const Encoder& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.totalPositions == rhs.totalPositions;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Encoder& encoder)
  {
    return os << "slaveIndex: " << encoder.slaveIndex << ", "
              << "totalPositions: " << encoder.totalPositions;
  }
};

} // namespace march
#endif  //  MARCH_HARDWARE_ENCODER_INCREMENTAL_H
