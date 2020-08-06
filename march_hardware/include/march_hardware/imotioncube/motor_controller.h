// Copyright 2019 Project March.

#include "actuation_mode.h"

namespace march
{
class MotorController
{
public:
  virtual double getAngleRadAbsolute() = 0;
  virtual double getAngleRadIncremental() = 0;
  virtual double getVelocityRadAbsolute() = 0;
  virtual double getVelocityRadIncremental() = 0;

  virtual ActuationMode getActuationMode() const = 0;

  virtual float getMotorCurrent() = 0;
  virtual float getControllerVoltage() = 0;
  virtual float getMotorVoltage() = 0;

  virtual void actuateRad(double target_rad) = 0;
  virtual void actuateTorque(int16_t target_torque) = 0;

  virtual void goToOperationEnabled() = 0;

  virtual bool initSdo(int cycle_time) = 0;

};

}  // namespace march
