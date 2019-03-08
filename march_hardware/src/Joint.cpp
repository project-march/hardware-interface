// Copyright 2019 Project March.
#include <ros/ros.h>

#include <march_hardware/Joint.h>

namespace march4cpp
{
Joint::Joint(std::string name, TemperatureGES temperatureGES, IMotionCube iMotionCube)
  : temperatureGES(temperatureGES), iMotionCube(iMotionCube)
{
  this->name = std::move(name);
}

Joint::Joint(std::string name, TemperatureGES temperatureGES) : temperatureGES(temperatureGES)
{
  this->name = std::move(name);
}
Joint::Joint(std::string name, IMotionCube iMotionCube) : iMotionCube(iMotionCube)

{
  this->name = std::move(name);
}

void Joint::initialize(int ecatCycleTime)
{
  if (hasIMotionCube())
  {
    iMotionCube.writeInitialSDOs(ecatCycleTime);
  }
  if (hasTemperatureGES())
  {
    temperatureGES.writeInitialSDOs(ecatCycleTime);
  }
}

void Joint::actuateRad(float targetPositionRad)
{
  // TODO(BaCo) check that the position is allowed and does not exceed (torque) limits.
  this->iMotionCube.actuateRad(targetPositionRad);
}

float Joint::getAngleRad()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("Joint %s has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleRad();
}

float Joint::getTemperature()
{
  if (!hasTemperatureGES())
  {
    ROS_WARN("Joint %s has no temperature sensor", this->name.c_str());
    return -1;
  }
  return this->temperatureGES.getTemperature();
}

int Joint::getTemperatureGESSlaveIndex()
{
  if (hasTemperatureGES())
  {
    return this->temperatureGES.getSlaveIndex();
  }
  return -1;
}

int Joint::getIMotionCubeSlaveIndex()
{
  if (hasIMotionCube())
  {
    return this->iMotionCube.getSlaveIndex();
  }
  return -1;
}

IMotionCube Joint::getIMotionCube()
{
  return this->iMotionCube;
}

std::string Joint::getName()
{
  return this->name;
}

bool Joint::hasIMotionCube()
{
  return this->iMotionCube.getSlaveIndex() != -1;
}

bool Joint::hasTemperatureGES()
{
  return this->temperatureGES.getSlaveIndex() != -1;
}
}  // namespace march4cpp