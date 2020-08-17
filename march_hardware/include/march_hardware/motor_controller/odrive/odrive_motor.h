#ifndef ODRIVE_INTERFACE_ODRIVE_MOTOR_H
#define ODRIVE_INTERFACE_ODRIVE_MOTOR_H

#include <utility>
#include <cmath>

#include "ros/ros.h"
#include "odrive.h"
#include "odrive_enums.h"
#include <march_hardware/motor_controller/actuation_mode.h>

static constexpr double PI_2 = 2 * M_PI;
static constexpr double MOTOR_KV = 100;
static constexpr double CURRENT_TO_TORQUE_CONVERSION = 8.27;

namespace march
{
class OdriveMotor : public Odrive
{
public:
  OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint, ActuationMode mode);

  bool initialize(int cycle_time) override;
  void prepareActuation() override;
  void reset() override;

  void actuateRad(double target_rad) override;
  void actuateTorque(int16_t target_torque) override;

  MotorControllerStates& getStates() override;

  float getMotorControllerVoltage() override;
  float getMotorCurrent() override;
  float getMotorVoltage() override;
  double getTorque() override;

  double getAngleRadAbsolute() override;
  double getVelocityRadAbsolute() override;

  double getAngleRadIncremental() override;
  double getVelocityRadIncremental() override;

  bool getIncrementalMorePrecise() const override;
  ActuationMode getActuationMode() const
  {
    return this->mode_;
  }
  int getSlaveIndex() const
  {
    return -1;
  }

private:
  std::string create_command(std::string command_name);
  int setState(uint8_t state);
  int getState();

  int getAngleCountsAbsolute();
  int getAngleCountsIncremental();

  ActuationMode mode_;
};

}  // namespace march
#endif  // ODRIVE_INTERFACE_ODRIVE_MOTOR_H
