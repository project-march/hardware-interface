// Copyright 2019 Project March.

#include "actuation_mode.h"
#include "motor_controller_state.h"
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
  virtual int16_t getTorque() = 0;

  virtual ActuationMode getActuationMode() const = 0;

  /**
   * Getter for the slave index.
   * @return slave index if the motor controller is an ethercat slave, else -1
   */
  virtual uint16_t getSlaveIndex() const = 0;

  virtual float getMotorCurrent() = 0;
  virtual float getMotorControllerVoltage() = 0;
  virtual float getMotorVoltage() = 0;

  /**
   * Get whether the incremental encoder is more precise than the absolute encoder
   * @return true if the incremental encoder has a higher resolution than the absolute encoder, false otherwise
   */
  virtual bool getIncrementalMorePrecise() const = 0;

  virtual void actuateRad(double target_rad) = 0;
  virtual void actuateTorque(int16_t target_torque) = 0;

  virtual void prepareActuation() = 0;
  virtual bool initialize(int cycle_time) = 0;
  virtual void reset() = 0;

  /**
   * Get the most recent states of the motor controller, i.e. all data that is read from the controller at every
   * communication cycle.
   * @return A MotorControllerState object containing all data read from the motor controller at every communication
   * cycle.
   */
  virtual MotorControllerStates getStates() = 0;
  /**
   * Check whether the motor controller is in an error state
   * @return false if the motor controller is in error state, otherwise true
   */
  virtual bool checkState() = 0;
  /**
   * Get a string describtion of the state and error states of the motor controller
   * @return string describing the current state as well as the error state(s) of the motor controller
   */
  virtual std::string getErrorStatus() = 0;

  virtual ~MotorController() noexcept = default;
};

}  // namespace march
