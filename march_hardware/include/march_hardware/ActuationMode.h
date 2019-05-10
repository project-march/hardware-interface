// Copyright 2018 Project March.

#ifndef MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H
#define MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H

#include <ros/package.h>

class ActuationMode
{
public:
  enum Value : int
  {
    position,
    torque,
    unknown,
  };

  ActuationMode() = default;

  explicit ActuationMode(const std::string& actuationMode)
  {
    if (actuationMode == "position")
    {
      this->value = position;
    }
    else if (actuationMode == "torque")
    {
      this->value = torque;
    }
    else
    {
      ROS_ASSERT_MSG(false, "Unknown actuation mode %s, setting to position mode", actuationMode.c_str());
      this->value = ActuationMode::unknown;
    }
  }

  int toModeNumber()
  {
    if (value == position)
    {
      return 8;
    }
    if (value == torque)
    {
      return 10;
    }
  }

  int getValue()
  {
    return value;
  }

  bool operator==(ActuationMode::Value a) const
  {
    return value == a;
  }

  bool operator!=(ActuationMode::Value a) const
  {
    return value != a;
  }

    inline const char* toString()
    {
        switch (this->value)
        {
            case torque:   return "torque";
            case position:   return "position";
            default:      return "[Unknown ActuationMode]";
        }
    }

private:
  Value value = unknown;
};

#endif  // MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H
