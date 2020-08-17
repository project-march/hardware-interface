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

  virtual bool checkState()
  {
    return true;
  }

  virtual std::string getErrorStatus()
  {
    std::ostringstream error_stream;
//    std::string state = IMCStateOfOperation(this->statusWord).getString().c_str();

    error_stream << "State: " << state;
    return error_stream.str();
  }
};

}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATES_H
