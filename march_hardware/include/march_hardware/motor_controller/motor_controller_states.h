// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H

#include <string>

namespace march
{
struct MotorControllerStates
{
public:
  MotorControllerStates() = default;

  float motor_current;
  float controller_voltage;
  float motor_voltage;
  int absolute_encoder_value;
  int incremental_encoder_value;
  double absolute_velocity;
  double incremental_velocity;

  /**
   * Check whether the motor controller is in an error state
   * @return false if the motor controller is in error state, otherwise true
   */
  virtual bool checkState() = 0;
  /**
   * Get a string description of the state and error states of the motor controller
   * @return string describing the current state as well as the error state(s) of the motor controller
   */
  virtual std::string getErrorStatus() = 0;
};

}  // namespace march

#endif  // MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
