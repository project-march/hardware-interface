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
  if (this->setState(States::AXIS_STATE_FULL_CALIBRATION_SEQUENCE) == 1)
  {
    ROS_FATAL("Calibration sequence was not finished successfully");
  }
}

// to be implemented
    void OdriveMotor::reset()
    {
        return;
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

    states.state = States(this->getState());

    return states;

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

int OdriveMotor::getState()
{
  int axis_state;
  std::string command_name_ = this->create_command(O_PM_CURRENT_STATE);
  if (this->read(command_name_, axis_state) == 1)
  {
    ROS_ERROR("Could not retrieve state of the motor");
    return ODRIVE_ERROR;
  }

  return axis_state;
}

}  // namespace march
