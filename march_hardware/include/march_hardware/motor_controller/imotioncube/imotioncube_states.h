// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_IMOTIONCUBE_STATES_H
#define MARCH_HARDWARE_IMOTIONCUBE_STATES_H

#include <string>
#include "march_hardware/motor_controller/motor_controller_states.h"
#include "imotioncube_state_of_operation.h"

namespace march
{
struct IMotionCubeStates : public MotorControllerStates
{
public:
  IMotionCubeStates() = default;

  uint16_t statusWord;
  std::string motionError;
  std::string detailedError;
  std::string secondDetailedError;
  IMCStateOfOperation state;
  std::string detailedErrorDescription;
  std::string motionErrorDescription;
  std::string secondDetailedErrorDescription;

  virtual bool checkState()
  {
    return !(this->state == march::IMCStateOfOperation::FAULT);
  }

  virtual std::string getErrorStatus()
  {
    std::ostringstream error_stream;
    std::string state = IMCStateOfOperation(this->statusWord).getString().c_str();

    error_stream << "State: " << state << "\nMotion Error: " << this->motionErrorDescription << " ("
                 << this->motionError << ")\nDetailed Error: " << this->detailedErrorDescription << " ("
                 << this->detailedError << ")\nSecond Detailed Error: " << this->secondDetailedErrorDescription << " ("
                 << this->secondDetailedError << ")";
    return error_stream.str();
  }
};

}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATES_H
