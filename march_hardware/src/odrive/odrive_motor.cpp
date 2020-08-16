#include "march_hardware/motor_controller/odrive/odrive_motor.h"

#include <utility>

namespace march
{
OdriveMotor::OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint,
                         ActuationMode mode)
  : Odrive(axisNumber, std::move(odriveEndpoint), false)
{
  this->mode = mode;
}

int OdriveMotor::get_position()
{
  int position;
  std::string parameter_name = this->axis_number + PM_ENCODER_COUNTS;

  this->read(parameter_name, position);
  return position;
}

float OdriveMotor::get_input_voltage()
{
  float odrive_input_voltage;

  this->read(ODRIVE_INPUT_VOLTAGE, odrive_input_voltage);
  return odrive_input_voltage;
}

}  // namespace march
