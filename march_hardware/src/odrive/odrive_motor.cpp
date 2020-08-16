#include "march_hardware/motor_controller/odrive/odrive_motor.h"

#include <utility>

namespace march
{
OdriveMotor::OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint,
                         ActuationMode mode)
  : Odrive(axisNumber, std::move(odriveEndpoint), false), mode(mode)
{
}

int OdriveMotor::initialize()
{
  if (this->setState(States::AXIS_STATE_FULL_CALIBRATION_SEQUENCE) == 1)
  {
    ROS_ERROR("Calibration sequence was not finished successfully");
    return ODRIVE_ERROR;
  }

  return ODRIVE_OK;
}

int OdriveMotor::setState(uint8_t state)
{
  std::string command_name_ = this->create_command(O_PM_REQUEST_STATE);
  if (this->write(command_name_, state) == 1)
  {
    ROS_ERROR("Could net set state; %i to the axis", state);
    return ODRIVE_ERROR;
  }
  return ODRIVE_OK
}
int OdriveMotor::getState()
{
  int axis_state;
  std::string command_name_ = this->create_command(O_PM_CURRENT_STATE);
  if (this->read(command_name_, axis_state) == 1)
  {
    ROS_ERROR("Could not retrieve incremental position of the encoder");
    return ODRIVE_ERROR;
  }

  return axis_state;
}

float OdriveMotor::getMotorControllerVoltage()
{
  float motor_controller_voltage;
  std::string command_name_ = this->create_command(O_PM_ODRIVE_INPUT_VOLTAGE);
  if (this->read(command_name_, motor_controller_voltage) == 1)
  {
    ROS_ERROR("Could not retrieve incremental position of the encoder");
    return ODRIVE_ERROR;
  }
  return motor_controller_voltage;
}
float OdriveMotor::getMotorCurrent()
{
  float motor_current;
  std::string command_name_ = this->create_command(O_PM_ACTUAL_MOTOR_CURRENT);
  if (this->read(command_name_, motor_current) == 1)
  {
    ROS_ERROR("Could not retrieve incremental position of the encoder");
    return ODRIVE_ERROR;
  }

  return motor_current;
}
float OdriveMotor::getTorque()
{
  float motor_current = this->getMotorCurrent();
  float motor_torque = CURRENT_TO_TORQUE_CONVERSION * motor_current / MOTOR_KV;

  return motor_torque;
}

float OdriveMotor::getAngleRadAbsolute()
{
  return 0;
}
float OdriveMotor::getVelocityRadAbsolute()
{
  return 0;
}

float OdriveMotor::getAngleRadIncremental()
{
  float iu_position;
  std::string command_name_ = this->create_command(O_PM_ENCODER_POSITION_UI);
  if (this->read(command_name_, iu_position) == 1)
  {
    ROS_ERROR("Could not retrieve incremental position of the encoder");
    return ODRIVE_ERROR;
  }

  float angle_rad = iu_position * PI_2 / std::pow(2, 12);
  return angle_rad;
}
float OdriveMotor::getVelocityRadIncremental()
{
  float iu_velocity;
  std::string command_name_ = this->create_command(O_PM_ENCODER_VELOCITY_UI);
  if (this->read(command_name_, iu_velocity) == 1)
  {
    ROS_ERROR("Could not retrieve incremental position of the encoder");
    return ODRIVE_ERROR;
  }

  float angle_rad = iu_velocity * PI_2 / std::pow(2, 12);
  return angle_rad;
}

std::string OdriveMotor::create_command(std::string command_name)
{
  if (command_name.at(0) == '.')
  {
    return this->axis_number + command_name;
  }
  return command_name;
}

}  // namespace march
