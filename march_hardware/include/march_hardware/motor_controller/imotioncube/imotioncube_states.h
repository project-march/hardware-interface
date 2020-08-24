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

  uint16_t status_word;
  std::string motion_error;
  std::string detailed_error;
  std::string second_detailed_error;
  IMCStateOfOperation state;
  std::string detailed_error_description;
  std::string motion_error_description;
  std::string second_detailed_error_description;

  bool checkState() override
  {
    return !(this->state == march::IMCStateOfOperation::FAULT);
  }

  std::string getErrorStatus() override
  {
    std::ostringstream error_stream;
    std::string state = IMCStateOfOperation(this->status_word).getString();

    error_stream << "State: " << state << "\nMotion Error: " << this->motion_error_description << " ("
                 << this->motion_error << ")\nDetailed Error: " << this->detailed_error_description << " ("
                 << this->detailed_error << ")\nSecond Detailed Error: " << this->second_detailed_error_description
                 << " (" << this->second_detailed_error << ")";
    return error_stream.str();
  }
};

}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATES_H
