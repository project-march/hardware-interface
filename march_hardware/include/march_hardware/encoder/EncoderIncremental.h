// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_INCREMENTAL_H
#define MARCH_HARDWARE_ENCODER_INCREMENTAL_H
#include "march_hardware/encoder/Encoder.h"

#include <ostream>

namespace march
{
class EncoderIncremental : public Encoder
{
public:
  explicit EncoderIncremental(size_t number_of_bits);

  double getAngleRad(uint8_t byte_offset);

  double toRad(int32_t iu);

  /** @brief Override comparison operator */
  friend bool operator==(const EncoderIncremental& lhs, const EncoderIncremental& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && lhs.getTotalPositions() == rhs.getTotalPositions();
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const EncoderIncremental& encoder)
  {
    return os << "slaveIndex: " << encoder.getSlaveIndex() << ", "
              << "totalPositions: " << encoder.getTotalPositions();
  }
};

}  // namespace march
#endif  //  MARCH_HARDWARE_ENCODER_INCREMENTAL_H
