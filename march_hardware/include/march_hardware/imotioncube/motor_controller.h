// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_IMOTIONCUBE_H
#define MARCH_HARDWARE_IMOTIONCUBE_H
#include "actuation_mode.h"
#include "march_hardware/ethercat/pdo_map.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/ethercat/slave.h"
#include "imotioncube_state.h"
#include "imotioncube_target_state.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"

#include <memory>
#include <unordered_map>
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

  virtual ActuationMode getActuationMode() const = 0;

  virtual float getMotorCurrent() = 0;
  virtual float getControllerVoltage() = 0;
  virtual float getMotorVoltage() = 0;

  virtual void actuateRad(double target_rad) = 0;
  virtual void actuateTorque(int16_t target_torque) = 0;

  virtual void goToOperationEnabled() = 0;

  virtual bool initSdo(SdoSlaveInterface& sdo, int cycle_time) = 0;

};

}  // namespace march
#endif  // MARCH_HARDWARE_IMOTIONCUBE_H
