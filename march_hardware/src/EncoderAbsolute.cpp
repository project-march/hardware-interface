// Copyright 2019 Project March.

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/EncoderAbsolute.h>
#include <cmath>
#include <ros/ros.h>

namespace march
{
EncoderAbsolute::EncoderAbsolute(int numberOfBits, int minPositionIU, int maxPositionIU, int zeroPositionIU,
                                 float safetyMarginRad)
  : Encoder(numberOfBits)
{
  ROS_ASSERT_MSG(safetyMarginRad >= 0, "SafetyMarginRad %f is below zero", safetyMarginRad);

  this->safetyMarginRad = safetyMarginRad;
  this->upperHardLimitIU = maxPositionIU;
  this->lowerHardLimitIU = minPositionIU;
  this->zeroPositionIU = zeroPositionIU;
  int safetyMarginIU = RadtoIU(safetyMarginRad) - this->zeroPositionIU;
  this->upperSoftLimitIU = this->upperHardLimitIU - safetyMarginIU;
  this->lowerSoftLimitIU = this->lowerHardLimitIU + safetyMarginIU;

  ROS_ASSERT_MSG(this->lowerSoftLimitIU < this->upperSoftLimitIU,
                 "Invalid range of motion. Safety margin too large or "
                 "min/max position invalid. lowerSoftLimit: %i IU, upperSoftLimit: "
                 "%i IU. lowerHardLimit: %i IU, upperHardLimit %i IU. safetyMargin: %f rad = %i IU",
                 this->lowerSoftLimitIU, this->upperSoftLimitIU, this->lowerHardLimitIU, this->upperHardLimitIU,
                 this->safetyMarginRad, safetyMarginIU);
}

float EncoderAbsolute::getAngleRad(uint8_t ActualPositionByteOffset)
{
  return IUtoRad(getAngleIU(ActualPositionByteOffset));
}

int EncoderAbsolute::RadtoIU(float rad)
{
  return static_cast<int>(rad * totalPositions / (2 * M_PI) + zeroPositionIU);
}

float EncoderAbsolute::IUtoRad(int iu)
{
  return static_cast<float>(iu - zeroPositionIU) * 2 * M_PI / totalPositions;
}

bool EncoderAbsolute::isWithinHardLimitsIU(int positionIU)
{
  return (positionIU > this->lowerHardLimitIU && positionIU < this->upperHardLimitIU);
}

bool EncoderAbsolute::isWithinSoftLimitsIU(int positionIU)
{
  return (positionIU > this->lowerSoftLimitIU && positionIU < this->upperSoftLimitIU);
}

bool EncoderAbsolute::isValidTargetIU(int currentIU, int targetIU)
{
  if (this->isWithinSoftLimitsIU(targetIU))
  {
    return true;
  }

  if (currentIU >= this->getUpperSoftLimitIU())
  {
    return (targetIU <= currentIU) && (targetIU > this->getLowerSoftLimitIU());
  }

  if (currentIU <= this->getLowerSoftLimitIU())
  {
    return (targetIU >= currentIU) && (targetIU < this->getUpperSoftLimitIU());
  }

  return false;
}

int EncoderAbsolute::getUpperSoftLimitIU() const
{
  return upperSoftLimitIU;
}

int EncoderAbsolute::getLowerSoftLimitIU() const
{
  return lowerSoftLimitIU;
}

int EncoderAbsolute::getUpperHardLimitIU() const
{
  return upperHardLimitIU;
}

int EncoderAbsolute::getLowerHardLimitIU() const
{
  return lowerHardLimitIU;
}
}  //  namespace march
