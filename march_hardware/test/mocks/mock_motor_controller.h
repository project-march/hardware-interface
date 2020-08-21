#pragma once
#include "mock_absolute_encoder.h"
#include "mock_incremental_encoder.h"
#include "mock_slave.h"

#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller_states.h"

#include <memory>

#include <gmock/gmock.h>

class MockMotorController : public march::MotorController
{
public:
  MOCK_METHOD0(prepareActuation, void());
  MOCK_CONST_METHOD0(getActuationMode, march::ActuationMode());

  MOCK_CONST_METHOD0(getSlaveIndex, int());
  MOCK_CONST_METHOD0(getIncrementalMorePrecise, bool());
  MOCK_METHOD0(getStates, march::MotorControllerStates&());

  MOCK_METHOD0(getAngleRadIncremental, double());
  MOCK_METHOD0(getAngleRadAbsolute, double());
  MOCK_METHOD0(getVelocityRadIncremental, double());
  MOCK_METHOD0(getVelocityRadAbsolute, double());

  MOCK_METHOD0(getMotorControllerVoltage, float());
  MOCK_METHOD0(getMotorVoltage, float());
  MOCK_METHOD0(getMotorCurrent, float());
  MOCK_METHOD0(getTorque, double());

  MOCK_METHOD1(actuateRad, void(double));
  MOCK_METHOD1(actuateTorque, void(double));

  MOCK_METHOD1(initialize, bool(int cycle_time));
  MOCK_METHOD0(reset, void());
};
