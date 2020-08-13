// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_STATE_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_STATE_H

#include <string>
#include "imotioncube/imotioncube_state.h"

namespace march
{
struct MotorControllerStates
{
public:
  MotorControllerStates() = default;

  float motorCurrent;
  float controllerVoltage;
  float motorVoltage;
  int absoluteEncoderValue;
  int incrementalEncoderValue;
  double absoluteVelocity;
  double incrementalVelocity;
  std::string errorStatus;
};

}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATE_H
