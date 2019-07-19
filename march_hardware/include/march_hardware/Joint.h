// Copyright 2019 Project March.
#ifndef MARCH4CPP__JOINT_H
#define MARCH4CPP__JOINT_H

#include <string>
#include <vector>

#include <march_hardware/IMotionCube.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <march_hardware/TemperatureGES.h>
#include <march_hardware/ActuationMode.h>
#include <march_hardware/IMotionCubeState.h>

namespace march4cpp
{

class Joint
{
private:
  std::string name;
  // TODO(bart) review this comment if it still makes sense
  /**
   * An explicit difference is made between allowActuation and actuationMode, since the situation can occur that
   * controllers has to be tested, but the joint should not actuate. For example to see how it responds to the error.
   * Both the actuationmode and allowactuation are defined in the .yaml in the hardware builder.
   */
  bool allowActuation;
  // Set this number via the hardware builder
  int netNumber = -1;
  std::string actuationModeName;
  ActuationMode actuationMode;
  IMotionCube iMotionCube;
  TemperatureGES temperatureGES;

public:
  // TODO(Tim) pass by reference or pointer instead of making copy
  // TODO refactor to using proper initialization lists
  Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, IMotionCube iMotionCube,
        std::string actuationmode);
  Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, IMotionCube iMotionCube, int netNumber,
        std::string actuationmode);
  Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, std::string actuationmode);
  Joint(std::string name, bool allowActuation, IMotionCube iMotionCube, std::string actuationmode);
  Joint(std::string name, bool allowActuation, IMotionCube iMotionCube, int netNumber, std::string actuationmode);

  void initialize(int ecatCycleTime);
  void prepareActuation();
  // TODO(Martijn) Refactor this to make joint less dependent on knowledge of the IMC
  void resetIMotionCube();

  void actuateRad(float targetPositionRad);
  void actuateTorque(int targetTorque);
  int getActuationMode();

  float getAngleRad();
  int getAngleIU();
  float getTorque();
  float getTemperature();
  IMotionCubeState getIMotionCubeState();

  std::string getName();
  int getTemperatureGESSlaveIndex();
  int getIMotionCubeSlaveIndex();
  int getNetNumber()
  {
    return netNumber;
  }

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
