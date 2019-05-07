// Copyright 2018 Project March.

#ifndef MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H
#define MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H

#include <ros/package.h>

class ActuationMode
{
public:
  enum Value : int
  {
    position_mode,
    torque_mode,
  };

  ActuationMode() = default;

  explicit ActuationMode(const std::string& actuationMode)
  {
    if (actuationMode == "position_mode")
    {
      this->value = position_mode;
    }
    else if (actuationMode == "torque_mode")
    {
      this->value = torque_mode;
    }
    else
    {
      ROS_ASSERT_MSG(false, "Unknown actuation mode %s, setting to position mode", actuationMode.c_str());
      this->value = ActuationMode::position_mode;
    }
  }

  int getValue(){
      return value;
  }

    int toModeNumber(){

  }

    bool operator==(ActuationMode::Value a) const
    {
      return value == a;
    }

    bool operator!=(ActuationMode::Value a) const
    {
      return value != a;
    }

private:
  Value value;
};

#endif  // MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H
