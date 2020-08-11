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
  virtual MotorControllerStates getStates() = 0;

  virtual ActuationMode getActuationMode() const = 0;
  virtual uint16_t getSlaveIndex() const = 0;

  virtual float getMotorCurrent() = 0;
  virtual float getMotorControllerVoltage() = 0;
  virtual float getMotorVoltage() = 0;
  virtual bool getIncrementalMorePrecise() const = 0;

  virtual void actuateRad(double target_rad) = 0;
  virtual void actuateTorque(int16_t target_torque) = 0;

  virtual void goToOperationEnabled() = 0;

  virtual bool initialize(int cycle_time) = 0;
  virtual void reset() = 0;
  virtual bool checkState(std::ostringstream& error_msg, std::string joint_name) = 0;

  virtual ~MotorController() noexcept = default;
};

}  // namespace march
