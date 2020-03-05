// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_INCREMENTAL_ENCODER_H
#define MARCH_HARDWARE_INCREMENTAL_ENCODER_H
#include "march_hardware/encoder/Encoder.h"

#include <ostream>

namespace march
{
class IncrementalEncoder : public Encoder
{
public:
  IncrementalEncoder(size_t number_of_bits, double transmission);

  double getAngleRad(uint8_t byte_offset);

  double toRad(int32_t iu);

  /** @brief Override comparison operator */
  friend bool operator==(const IncrementalEncoder& lhs, const IncrementalEncoder& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && lhs.getTotalPositions() == rhs.getTotalPositions();
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const IncrementalEncoder& encoder)
  {
    return os << "slaveIndex: " << encoder.getSlaveIndex() << ", "
              << "totalPositions: " << encoder.getTotalPositions();
  }

private:
  const double transmission_;
};
}  // namespace march

#endif  // MARCH_HARDWARE_INCREMENTAL_ENCODER_H
