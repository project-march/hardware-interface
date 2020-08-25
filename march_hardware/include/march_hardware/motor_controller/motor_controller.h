// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_H
#include "actuation_mode.h"
#include "motor_controller_states.h"
#include <string>

namespace march
{
class MotorController
{
public:
  virtual double getAngleRadAbsolute() = 0;
  virtual double getAngleRadIncremental() = 0;
  virtual double getVelocityRadAbsolute() = 0;
  virtual double getVelocityRadIncremental() = 0;
  virtual double getTorque() = 0;

  virtual ActuationMode getActuationMode() const = 0;

  /**
   * Getter for the slave index.
   * @return slave index if the motor controller is an ethercat slave, else -1
   */
  virtual int getSlaveIndex() const = 0;

  virtual float getMotorCurrent() = 0;
  virtual float getMotorControllerVoltage() = 0;
  virtual float getMotorVoltage() = 0;

  /**
   * Get whether the incremental encoder is more precise than the absolute encoder
   * @return true if the incremental encoder has a higher resolution than the absolute encoder, false otherwise
   */
  virtual bool getIncrementalMorePrecise() const = 0;

  virtual void actuateRad(double target_rad) = 0;
  virtual void actuateTorque(double target_torque_ampere) = 0;

  virtual void prepareActuation() = 0;
  virtual void reset() = 0;

  /**
   * Get the most recent states of the motor controller, i.e. all data that is read from the controller at every
   * communication cycle.
   * @return A MotorControllerState object containing all data read from the motor controller at every communication
   * cycle.
   */
  virtual std::unique_ptr<MotorControllerStates> getStates() = 0;

  virtual ~MotorController() noexcept = default;
};

}  // namespace march
#endif  // MARCH_HARDWARE_MOTOR_CONTROLLER_H
