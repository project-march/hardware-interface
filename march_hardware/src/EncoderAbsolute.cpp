// Copyright 2019 Project March.

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/EncoderAbsolute.h>
#include <cmath>
#include <ros/ros.h>

namespace march
{
EncoderAbsolute::EncoderAbsolute(int numberOfBits, int minPositionIU, int maxPositionIU, 
                                  int zeroPositionIU, float safetyMarginRad) : Encoder(numberOfBits)
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

bool Encoder::isWithinHardLimitsIU(int positionIU)
{
  return (positionIU > this->lowerHardLimitIU && positionIU < this->upperHardLimitIU);
}

bool Encoder::isWithinSoftLimitsIU(int positionIU)
{
  return (positionIU > this->lowerSoftLimitIU && positionIU < this->upperSoftLimitIU);
}

bool Encoder::isValidTargetIU(int currentIU, int targetIU)
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

int Encoder::getUpperSoftLimitIU() const
{
  return upperSoftLimitIU;
}

int Encoder::getLowerSoftLimitIU() const
{
  return lowerSoftLimitIU;
}

int Encoder::getUpperHardLimitIU() const
{
  return upperHardLimitIU;
}

int Encoder::getLowerHardLimitIU() const
{
  return lowerHardLimitIU;
}
} //  namespace march
