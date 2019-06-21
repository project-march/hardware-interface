// Copyright 2019 Project March.
#include <ros/ros.h>

#include <march_hardware/Joint.h>

namespace march4cpp
{
// ImotionCube and Temperature GES
Joint::Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, IMotionCube iMotionCube,
             std::string actuationModeName)
  : temperatureGES(temperatureGES), iMotionCube(iMotionCube), actuationMode(ActuationMode(actuationModeName))
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}

// ImotionCube, Temperature GES and PDB
Joint::Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, IMotionCube iMotionCube,
             int netNumber, std::string actuationModeName)
  : temperatureGES(temperatureGES), iMotionCube(iMotionCube), netNumber(netNumber), actuationMode(ActuationMode(actuationModeName)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}

// Only Temperature GES
//TODO:(bart) make sure these constructors make sense with actuationmode
Joint::Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, std::string actuationModeName) : temperatureGES(temperatureGES), actuationMode(ActuationMode(actuationModeName)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}

// Only ImotionCube
Joint::Joint(std::string name, bool allowActuation, IMotionCube iMotionCube, std::string actuationModeName) : iMotionCube(iMotionCube), actuationMode(ActuationMode(actuationModeName)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}

// ImotionCube and PDB
Joint::Joint(std::string name, bool allowActuation, IMotionCube iMotionCube, int netNumber, std::string actuationModeName)
  : iMotionCube(iMotionCube), netNumber(netNumber), actuationMode(ActuationMode(actuationModeName)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}

void Joint::initialize(int ecatCycleTime)
{
  if (hasIMotionCube())
  {
    iMotionCube.writeInitialSDOs(ecatCycleTime, this->actuationMode);
  }
  if (hasTemperatureGES())
  {
    temperatureGES.writeInitialSDOs(ecatCycleTime);
  }
}

void Joint::prepareActuation()
{
  if (this->allowActuation)
  {
    this->iMotionCube.goToOperationEnabled();
  }
  else
  {
    ROS_ERROR("Trying to prepare joint %s for actuation while it is not "
              "allowed to actuate",
              this->name.c_str());
  }
}

void Joint::actuateRad(float targetPositionRad)
{
  ROS_ASSERT_MSG(this->allowActuation, "Joint %s is not allowed to actuate, "
                                       "yet its actuate method has been "
                                       "called.",
                 this->name.c_str());
  // TODO(BaCo) check that the position is allowed and does not exceed (torque)
  // limits.
  this->iMotionCube.actuateRad(targetPositionRad);
}

void Joint::actuateCurrent(float targetCurrent)
{
  // TODO(BaCo) check that the position is allowed and does not exceed (torque) limits.
  this->iMotionCube.actuateCurrent(targetCurrent);
}

int Joint::getActuationMode()
{
  return this->actuationMode.getValue();
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

float Joint::getTorque()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("Joint %s has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getTorque();
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

bool Joint::canActuate()
{
  return this->allowActuation;
}
}  // namespace march4cpp
