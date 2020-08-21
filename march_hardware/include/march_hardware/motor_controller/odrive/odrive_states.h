// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_IMOTIONCUBE_STATES_H
#define MARCH_HARDWARE_IMOTIONCUBE_STATES_H

#include <string>
#include "march_hardware/motor_controller/motor_controller_states.h"
#include "odrive_enums.h"

namespace march
{
struct OdriveStates : public MotorControllerStates
{
public:
  OdriveStates() = default;

  States state;
  uint16_t axisError;
  uint16_t axisMotorError;
  uint16_t axisEncoderError;
  uint16_t axisControllerError;

  std::string axisErrorDescription;
  std::string axisMotorErrorDescription;
  std::string axisEncoderErrorDescription;
  std::string axisControllerErrorDescription;

  bool checkState() override
  {
    if (this->axisError == ERROR_NONE)
    {
      return true;
    }
    return false;
  }

  std::string getErrorStatus() override
  {
    std::ostringstream error_stream;

    error_stream << "State: " << this->state << "\nAxis Error: " << this->axisError << " ("
                 << this->axisErrorDescription << ")"
                 << "\nMotor Error: " << this->axisMotorError << " (" << this->axisMotorErrorDescription << ")"
                 << "\nEncoder Error: " << this->axisEncoderError << " (" << this->axisEncoderErrorDescription << ")"
                 << "\nController Error: " << this->axisControllerError << " (" << this->axisControllerErrorDescription
                 << ")";

    return error_stream.str();
  }
};

}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATES_H
