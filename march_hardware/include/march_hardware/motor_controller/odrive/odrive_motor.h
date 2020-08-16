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

  int initialize();

  int setState(uint8_t state);
  int getState();

  float getMotorControllerVoltage();
  float getMotorCurrent();
  float getTorque();

  float getAngleRadAbsolute();
  float getVelocityRadAbsolute();

  float getAngleRadIncremental();
  float getVelocityRadIncremental();

private:
  std::string create_command(std::string command_name);
  ActuationMode mode;
};

}  // namespace march
#endif  // ODRIVE_INTERFACE_ODRIVE_MOTOR_H
