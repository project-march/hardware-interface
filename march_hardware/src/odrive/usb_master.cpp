// Copyright 2019 Project March.
#include "march_hardware/motor_controller/odrive/usb_master.h"
#include <ros/ros.h>

namespace march
{
std::shared_ptr<OdriveEndpoint> UsbMaster::getSerialConnection(const std::string& serial_number)
{
  for (uint i = 0; i < this->odrive_endpoints_.size(); ++i)
  {
    if (this->odrive_endpoints_[i]->getSerialNumber().compare(serial_number) == 0)
    {
      return this->odrive_endpoints_[i];
    }
  }

  std::shared_ptr<OdriveEndpoint> odrive_endpoint = std::make_shared<OdriveEndpoint>();
  ROS_INFO("Open serial connection %s", serial_number.c_str());
  odrive_endpoint->open_connection(serial_number);

  this->odrive_endpoints_.push_back(odrive_endpoint);

  return odrive_endpoint;
}
}  // namespace march
