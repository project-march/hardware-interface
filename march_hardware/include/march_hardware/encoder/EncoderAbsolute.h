// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_ABSOLUTE_H
#define MARCH_HARDWARE_ENCODER_ABSOLUTE_H
#include "march_hardware/encoder/Encoder.h"

#include <ostream>

namespace march
{
class EncoderAbsolute : public Encoder
{
public:
  EncoderAbsolute(size_t number_of_bits, int32_t lower_limit_iu, int32_t upper_limit_iu, double lower_limit_rad,
                  double upper_limit_rad, double lower_soft_limit_rad, double upper_soft_limit_rad);

  /*
   * Reads out the encoder from the slave and transforms the result to radians.
   * @param actual_position_byte_offset the byte offset in the slave register for the IU position
   * @returns The current position of the encoder in radians
   */
  double getAngleRad(uint8_t byte_offset) const;

  /*
   * Converts encoder Internal Units (IU) to radians.
   */
  double toRad(int32_t iu) const;

  /*
   * Converts radians to encoder Internal Units (IU).
   */
  int32_t fromRad(double rad) const;

  bool isWithinHardLimitsIU(int32_t iu) const;
  bool isWithinSoftLimitsIU(int32_t iu) const;
  bool isValidTargetIU(int32_t current_iu, int32_t target_iu) const;

  int32_t getUpperSoftLimitIU() const;
  int32_t getLowerSoftLimitIU() const;
  int32_t getUpperHardLimitIU() const;
  int32_t getLowerHardLimitIU() const;

  /** @brief Override comparison operator */
  friend bool operator==(const EncoderAbsolute& lhs, const EncoderAbsolute& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && lhs.getTotalPositions() == rhs.getTotalPositions() &&
           lhs.upper_soft_limit_iu_ == rhs.upper_soft_limit_iu_ &&
           lhs.lower_soft_limit_iu_ == rhs.lower_soft_limit_iu_ && lhs.upper_limit_iu_ == rhs.upper_limit_iu_ &&
           lhs.lower_limit_iu_ == rhs.lower_limit_iu_ && lhs.zero_position_iu_ == rhs.zero_position_iu_;
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const EncoderAbsolute& encoder)
  {
    return os << "slaveIndex: " << encoder.getSlaveIndex() << ", "
              << "totalPositions: " << encoder.getTotalPositions() << ", "
              << "upperHardLimit: " << encoder.upper_limit_iu_ << ", "
              << "lowerHardLimit: " << encoder.lower_limit_iu_ << ", "
              << "upperSoftLimit: " << encoder.upper_soft_limit_iu_ << ", "
              << "lowerSoftLimit: " << encoder.lower_soft_limit_iu_ << ", "
              << "zeroPositionIU: " << encoder.zero_position_iu_;
  }

  static constexpr double MAX_RANGE_DIFFERENCE = 0.05;

private:
  int32_t zero_position_iu_ = 0;
  int32_t lower_limit_iu_ = 0;
  int32_t upper_limit_iu_ = 0;
  int32_t lower_soft_limit_iu_ = 0;
  int32_t upper_soft_limit_iu_ = 0;
};
}  // namespace march

#endif  // MARCH_HARDWARE_ENCODER_ABSOLUTE_H
