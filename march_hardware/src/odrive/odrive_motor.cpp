#include "march_hardware/motor_controller/odrive/odrive_motor.h"
#include "march_hardware/motor_controller/odrive/odrive_states.h"

#include <utility>

namespace march
{
OdriveMotor::OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint,
                         ActuationMode mode)
  : Odrive(axisNumber, std::move(odriveEndpoint), false), mode_(mode)
{
}

bool OdriveMotor::initialize(int cycle_time)
{
  return cycle_time;
}

void OdriveMotor::prepareActuation()
{
  this->importOdriveJson();

  if (this->setState(States::AXIS_STATE_FULL_CALIBRATION_SEQUENCE) == 1)
  {
    ROS_FATAL("Calibration sequence was not finished successfully");
    return;
  }

  this->waitForIdleState();

  if (this->setState(States::AXIS_STATE_CLOSED_LOOP_CONTROL) == 1)
  {
    ROS_FATAL("Calibration sequence was not finished successfully");
    return;
  }
}

bool OdriveMotor::waitForIdleState(float timeout)
{
  float current_time = 0;
  while (this->getState() != States::AXIS_STATE_IDLE)
  {
    ros::Duration(0.5).sleep();
    current_time += 0.5;

    if (current_time == timeout)
    {
      ROS_FATAL("Odrive axis did not return to idle state, current state is %i", this->getState());
      return false;
    }
  }
  return true;
}

// to be implemented
void OdriveMotor::reset()
{
  uint16_t axis_error = 0;
  std::string command_name_ = this->create_command(O_PM_AXIS_ERROR);
  if (this->write(command_name_, axis_error) == 1)
  {
    ROS_ERROR("Could not reset axis");
  }
}

void OdriveMotor::actuateRad(double target_rad)
{
  ROS_INFO("Actuating rad %f", target_rad);
  return;
}

void OdriveMotor::actuateTorque(int16_t target_torque)
{
  ROS_INFO("Actuating torque %d", target_torque);
  return;
}

MotorControllerStates& OdriveMotor::getStates()
{
  static OdriveStates states;

  // Common states
  states.motorCurrent = this->getMotorCurrent();
  states.controllerVoltage = this->getMotorControllerVoltage();
  states.motorVoltage = this->getMotorVoltage();

  states.absoluteEncoderValue = this->getAngleCountsAbsolute();
  states.incrementalEncoderValue = this->getAngleCountsIncremental();
  states.absoluteVelocity = this->getVelocityRadAbsolute();
  states.incrementalVelocity = this->getVelocityRadIncremental();

  states.axisError = this->getAxisError();
  states.axisMotorError = this->getAxisMotorError();
  states.axisEncoderError = this->getAxisEncoderError();
  states.axisControllerError = this->getAxisControllerError();

  states.state = States(this->getState());

  return states;
}

uint16_t OdriveMotor::getAxisError()
{
  uint16_t axis_error;
  std::string command_name_ = this->create_command(O_PM_AXIS_ERROR);
  if (this->read(command_name_, axis_error) == 1)
  {
    ROS_ERROR("Could not retrieve axis error");
    return ODRIVE_ERROR;
  }

  return axis_error;
}

uint16_t OdriveMotor::getAxisMotorError()
{
  uint16_t axis_motor_error;
  std::string command_name_ = this->create_command(O_PM_AXIS_MOTOR_ERROR);
  if (this->read(command_name_, axis_motor_error) == 1)
  {
    ROS_ERROR("Could not retrieve axis motor error");
    return ODRIVE_ERROR;
  }

  return axis_motor_error;
}

uint8_t OdriveMotor::getAxisEncoderError()
{
  uint8_t axis_encoder_error;
  std::string command_name_ = this->create_command(O_PM_AXIS_ENCODER_ERROR);
  if (this->read(command_name_, axis_encoder_error) == 1)
  {
    ROS_ERROR("Could not retrieve axis encoder error");
    return ODRIVE_ERROR;
  }

  return axis_encoder_error;
}

uint8_t OdriveMotor::getAxisControllerError()
{
  uint8_t axis_controller_error;
  std::string command_name_ = this->create_command(O_PM_AXIS_CONTROLLER_ERROR);
  if (this->read(command_name_, axis_controller_error) == 1)
  {
    ROS_ERROR("Could not retrieve axis controller error");
    return ODRIVE_ERROR;
  }

  return axis_controller_error;
}

float OdriveMotor::getMotorControllerVoltage()
{
  float motor_controller_voltage;
  std::string command_name_ = this->create_command(O_PM_ODRIVE_INPUT_VOLTAGE);
  if (this->read(command_name_, motor_controller_voltage) == 1)
  {
    ROS_ERROR("Could not retrieve motor controller voltage");
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
    ROS_ERROR("Could not retrieve motor courrent");
    return ODRIVE_ERROR;
  }

  return motor_current;
}

float OdriveMotor::getMotorVoltage()
{
  float motor_voltage;
  std::string command_name_ = this->create_command(O_PM_ACTUAL_MOTOR_VOLTAGE_D);
  if (this->read(command_name_, motor_voltage) == 1)
  {
    ROS_ERROR("Could not retrieve motor voltage");
    return ODRIVE_ERROR;
  }

  return motor_voltage;
}

double OdriveMotor::getTorque()
{
  double motor_current = this->getMotorCurrent();
  double motor_torque = CURRENT_TO_TORQUE_CONVERSION * motor_current / MOTOR_KV;

  return motor_torque;
}

int OdriveMotor::getAngleCountsAbsolute()
{
  return 0;
}

double OdriveMotor::getAngleRadAbsolute()
{
  double angle_rad = this->getAngleCountsAbsolute() * PI_2 / std::pow(2, 17);
  return angle_rad;
}

double OdriveMotor::getVelocityRadAbsolute()
{
  return 0;
}

bool OdriveMotor::getIncrementalMorePrecise() const
{
  return true;
}

int OdriveMotor::getAngleCountsIncremental()
{
  float iu_position;
  std::string command_name_ = this->create_command(O_PM_ENCODER_POSITION_UI);
  if (this->read(command_name_, iu_position) == 1)
  {
    ROS_ERROR("Could not retrieve incremental position of the encoder");
    return ODRIVE_ERROR;
  }
  return iu_position;
}

double OdriveMotor::getAngleRadIncremental()
{
  double angle_rad = this->getAngleCountsIncremental() * PI_2 / std::pow(2, 12);
  return angle_rad;
}

double OdriveMotor::getVelocityRadIncremental()
{
  float iu_velocity;
  std::string command_name_ = this->create_command(O_PM_ENCODER_VELOCITY_UI);
  if (this->read(command_name_, iu_velocity) == 1)
  {
    ROS_ERROR("Could not retrieve incremental velocity of the encoder");
    return ODRIVE_ERROR;
  }

  double angle_rad = iu_velocity * PI_2 / std::pow(2, 12);
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

uint8_t OdriveMotor::getState()
{
  uint8_t axis_state;
  std::string command_name_ = this->create_command(O_PM_CURRENT_STATE);
  if (this->read(command_name_, axis_state) == 1)
  {
    ROS_ERROR("Could not retrieve state of the motor");
    return ODRIVE_ERROR;
  }

  return axis_state;
}

}  // namespace march
