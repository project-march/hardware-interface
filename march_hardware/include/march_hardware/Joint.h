// Copyright 2019 Project March.
#ifndef MARCH4CPP__JOINT_H
#define MARCH4CPP__JOINT_H

#include <string>

#include <march_hardware/IMotionCube.h>
#include <march_hardware/TemperatureGES.h>
#include <march_hardware/ActuationMode.h>

namespace march4cpp
{
class Joint
{
private:
  std::string name;
  // TODO(bart) review this comment if it still makes sense
  /**
   * An explicit difference is made between allowActuation and actuationMode, since the situation can occur that
   * controllers want to be tested for example to see how it responds to error. For the sake of safety an explicit
   * statement in the .yaml will indicate what can be expected.
   */
  bool allowActuation;
  ActuationMode actuationMode;
  IMotionCube iMotionCube;
  TemperatureGES temperatureGES;

public:
  Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, IMotionCube iMotionCube,
                std::string actuationmode);
  Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES);
  Joint(std::string name, bool allowActuation, IMotionCube iMotionCube);

  void initialize(int ecatCycleTime);
  void prepareActuation();
  void actuateRad(float targetPositionRad);
  void actuateCurrent(float targetCurrentRad);
  int getActuationMode();

  float getAngleRad();
  float getTorque();
  float getTemperature();

  std::string getName();
  int getTemperatureGESSlaveIndex();
  int getIMotionCubeSlaveIndex();

  bool hasIMotionCube();
  bool hasTemperatureGES();
  bool canActuate();

  /** @brief Override comparison operator */
  friend bool operator==(const Joint& lhs, const Joint& rhs)
  {
    return lhs.name == rhs.name && lhs.iMotionCube == rhs.iMotionCube && lhs.temperatureGES == rhs.temperatureGES &&
           lhs.allowActuation == rhs.allowActuation && lhs.actuationMode.getValue() == rhs.actuationMode.getValue();
  }

  friend bool operator!=(const Joint& lhs, const Joint& rhs)
  {
    return !(lhs == rhs);
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
  {
    return os << "name: " << joint.name << ", "
            << "ActuationMode: " << joint.actuationMode.toString() << ", "
              << "allowActuation: " << joint.allowActuation << ", "
              << "imotioncube: " << joint.iMotionCube << ","
              << "temperatureges: " << joint.temperatureGES;
  }
};
}  // namespace march4cpp
#endif
