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
      //      ROS_ASSERT_MSG(false, "Unknown actuation mode %s, setting to unknown mode", actuationMode.c_str());
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

  int getValue() const
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

  std::string toString() const
  {
    switch (this->value)
    {
      case torque:
        return "torque";
      case position:
        return "position";
      default:
        return "unknown";
    }
  }

private:
  Value value = unknown;
};

#endif  // MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H
