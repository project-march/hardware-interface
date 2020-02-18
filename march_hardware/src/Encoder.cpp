// Copyright 2019 Project March.
#include "march_hardware/Encoder.h"
#include "march_hardware/EtherCAT/EthercatIO.h"
#include "march_hardware/error/hardware_exception.h"

#include <cmath>

#include <ros/ros.h>

namespace march
{
const double PI_2 = 6.28318530717958647692;

Encoder::Encoder(size_t number_of_bits, int32_t lower_limit_iu, int32_t upper_limit_iu, double lower_limit_rad,
                 double upper_limit_rad, double lower_soft_limit_rad, double upper_soft_limit_rad)
  : lower_limit_iu_(lower_limit_iu), upper_limit_iu_(upper_limit_iu)
{
  this->total_position_ = Encoder::calculateTotalPositions(number_of_bits);

  const double encoder_iu_per_rad = this->total_position_ / PI_2;
  const double iu_per_rad = (this->upper_limit_iu_ - this->lower_limit_iu_) / (upper_limit_rad - lower_limit_rad);
  const double difference = std::abs(encoder_iu_per_rad - iu_per_rad) / encoder_iu_per_rad;
  if (difference > Encoder::MAX_IU_PER_RAD_DIFFERENCE)
  {
    ROS_WARN("Difference %f%% exceeds %f%%\nEncoder IU per radians = %f\nLimits IU per radians = %f", difference * 100,
             Encoder::MAX_IU_PER_RAD_DIFFERENCE * 100, encoder_iu_per_rad, iu_per_rad);
  }

  this->zero_position_iu_ = this->lower_limit_iu_ - lower_limit_rad * this->total_position_ / PI_2;
  this->lower_soft_limit_iu_ = this->fromRad(lower_soft_limit_rad);
  this->upper_soft_limit_iu_ = this->fromRad(upper_soft_limit_rad);

  if (this->lower_limit_iu_ >= this->upper_limit_iu_ || this->lower_soft_limit_iu_ >= this->upper_soft_limit_iu_ ||
      this->lower_soft_limit_iu_ < this->lower_limit_iu_ || this->upper_soft_limit_iu_ > this->upper_limit_iu_)
  {
    throw error::HardwareException(error::ErrorType::INVALID_RANGE_OF_MOTION,
                                   "lower_soft_limit: %d IU, upper_soft_limit: %d IU\n"
                                   "lower_hard_limit: %d IU, upper_hard_limit: %d IU",
                                   this->lower_soft_limit_iu_, this->upper_soft_limit_iu_, this->lower_limit_iu_,
                                   this->upper_limit_iu_);
  }
}

double Encoder::getAngleRad(uint8_t actual_position_byte_offset) const
{
  return this->toRad(this->getAngleIU(actual_position_byte_offset));
}

int32_t Encoder::getAngleIU(uint8_t actual_position_byte_offset) const
{
  if (this->slave_index_ == -1)
  {
    ROS_FATAL("Encoder has slaveIndex of -1");
  }
  union bit32 return_byte = get_input_bit32(this->slave_index_, actual_position_byte_offset);
  ROS_DEBUG("Encoder read (IU): %d", return_byte.i);
  return return_byte.i;
}

int32_t Encoder::fromRad(double rad) const
{
  return (rad * this->total_position_ / PI_2) + this->zero_position_iu_;
}

double Encoder::toRad(int32_t iu) const
{
  return (iu - this->zero_position_iu_) * PI_2 / this->total_position_;
}

bool Encoder::isWithinHardLimitsIU(int32_t iu) const
{
  return (iu > this->lower_limit_iu_ && iu < this->upper_limit_iu_);
}

bool Encoder::isWithinSoftLimitsIU(int32_t iu) const
{
  return (iu > this->lower_soft_limit_iu_ && iu < this->upper_soft_limit_iu_);
}

bool Encoder::isValidTargetIU(int32_t current_iu, int32_t target_iu) const
{
  if (this->isWithinSoftLimitsIU(target_iu))
  {
    return true;
  }

  if (current_iu >= this->upper_soft_limit_iu_)
  {
    return (target_iu <= current_iu) && (target_iu > this->lower_soft_limit_iu_);
  }

  if (current_iu <= this->lower_soft_limit_iu_)
  {
    return (target_iu >= current_iu) && (target_iu < this->upper_soft_limit_iu_);
  }

  return false;
}

void Encoder::setSlaveIndex(int slave_index)
{
  this->slave_index_ = slave_index;
}

int32_t Encoder::getUpperSoftLimitIU() const
{
  return this->upper_soft_limit_iu_;
}

int32_t Encoder::getLowerSoftLimitIU() const
{
  return this->lower_soft_limit_iu_;
}

int32_t Encoder::getUpperHardLimitIU() const
{
  return this->upper_limit_iu_;
}

int32_t Encoder::getLowerHardLimitIU() const
{
  return this->lower_limit_iu_;
}

size_t Encoder::calculateTotalPositions(size_t number_of_bits)
{
  if (number_of_bits < Encoder::MIN_RESOLUTION || number_of_bits > Encoder::MAX_RESOLUTION)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ENCODER_RESOLUTION,
                                   "Encoder resolution of %d is not within range [%ld, %ld]", number_of_bits,
                                   Encoder::MIN_RESOLUTION, Encoder::MAX_RESOLUTION);
  }
  return std::pow(2, number_of_bits) - 1;
}

}  // namespace march
