#ifndef ODRIVE_INTERFACE_ODRIVE_MOTOR_H
#define ODRIVE_INTERFACE_ODRIVE_MOTOR_H

#include <utility>

#include "ros/ros.h"
#include "odrive.h"
#include "odrive_enums.h"
#include <march_hardware/motor_controller/actuation_mode.h>

namespace march
{
class OdriveMotor : public Odrive
{
public:
  OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint, ActuationMode mode);

  int get_position();

  float get_input_voltage();

  int get_axis_error();

private:
  ActuationMode mode;
};

}  // namespace march
#endif  // ODRIVE_INTERFACE_ODRIVE_MOTOR_H
