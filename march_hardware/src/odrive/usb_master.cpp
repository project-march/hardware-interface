// Copyright 2019 Project March.
#include "march_hardware/motor_controller/odrive/usb_master.h"
#include <ros/ros.h>

namespace march
{
std::shared_ptr<OdriveEndpoint> UsbMaster::getSerialConnection(const std::string& serial_number)
{
  for (uint i = 0; i < this->odrive_endpoints.size(); ++i)
  {
    if (odrive_endpoints[i]->getSerialNumber().compare(serial_number) == 0)
    {
      return odrive_endpoints[i];
    }
  }

  std::shared_ptr<OdriveEndpoint> odrive_endpoint = std::make_shared<OdriveEndpoint>();
  ROS_INFO("Open serial connection %s", serial_number);
  odrive_endpoint->open_connection(serial_number);

  odrive_endpoints.push_back(odrive_endpoint);
  odrive_endpoint->getSerialNumber();
  return odrive_endpoint;
}
}  // namespace march
