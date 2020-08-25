// Copyright 2018 Project March.
#include "march_hardware/joint.h"
#include "march_hardware/march_robot.h"
#include "march_hardware/temperature/temperature_sensor.h"
#include "march_hardware/error/hardware_exception.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace march
{
MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf, ::std::string ifName,
                       std::vector<std::shared_ptr<Slave>> slave_list, int ecatCycleTime, int ecatSlaveTimeout)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(ifName, slave_list, ecatCycleTime, ecatSlaveTimeout)
  , pdb_(nullptr)
{
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
                       std::shared_ptr<PowerDistributionBoard> powerDistributionBoard, ::std::string ifName,
                       std::vector<std::shared_ptr<Slave>> slave_list, int ecatCycleTime, int ecatSlaveTimeout)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(ifName, slave_list, ecatCycleTime, ecatSlaveTimeout)
  , pdb_(std::move(powerDistributionBoard))
{
}

void MarchRobot::startEtherCAT(bool reset_motor_controllers)
{
  if (ethercatMaster.isOperational())
  {
    ROS_WARN("Trying to start EtherCAT while it is already active.");
    return;
  }

  bool config_overwritten = ethercatMaster.start();

  if (reset_motor_controllers || config_overwritten)
  {
    ROS_DEBUG("Resetting all motor controllers due to either: reset arg: %d or downloading of configuration file: %d",
              reset_motor_controllers, config_overwritten);
    resetMotorControllers();

    ROS_INFO("Restarting the EtherCAT Master");
    ethercatMaster.stop();
    config_overwritten = ethercatMaster.start();
  }
}

void MarchRobot::stopEtherCAT()
{
  if (!ethercatMaster.isOperational())
  {
    ROS_WARN("Trying to stop EtherCAT while it is not active.");
    return;
  }

  ethercatMaster.stop();
}

void MarchRobot::resetMotorControllers()
{
  for (auto& joint : jointList)
  {
    joint.resetMotorController();
  }
}

bool MarchRobot::isEthercatOperational()
{
  return ethercatMaster.isOperational();
}

std::exception_ptr MarchRobot::getLastEthercatException() const noexcept
{
  return this->ethercatMaster.getLastException();
}

void MarchRobot::waitForPdo()
{
  this->ethercatMaster.waitForPdo();
}

int MarchRobot::getEthercatCycleTime() const
{
  return this->ethercatMaster.getCycleTime();
}

Joint& MarchRobot::getJoint(::std::string jointName)
{
  if (!ethercatMaster.isOperational())
  {
    ROS_WARN("Trying to access joints while ethercat is not operational. This "
             "may lead to incorrect sensor data.");
  }
  for (auto& joint : jointList)
  {
    if (joint.getName() == jointName)
    {
      return joint;
    }
  }

  throw std::out_of_range("Could not find joint with name " + jointName);
}

Joint& MarchRobot::getJoint(size_t index)
{
  if (!ethercatMaster.isOperational())
  {
    ROS_WARN("Trying to access joints while ethercat is not operational. This "
             "may lead to incorrect sensor data.");
  }
  return this->jointList.at(index);
}

size_t MarchRobot::size() const
{
  return this->jointList.size();
}

MarchRobot::iterator MarchRobot::begin()
{
  if (!ethercatMaster.isOperational())
  {
    ROS_WARN("Trying to access joints while ethercat is not operational. This "
             "may lead to incorrect sensor data.");
  }
  return this->jointList.begin();
}

MarchRobot::iterator MarchRobot::end()
{
  return this->jointList.end();
}

bool MarchRobot::hasPowerDistributionboard() const
{
  return this->pdb_ != nullptr;
}

PowerDistributionBoard* MarchRobot::getPowerDistributionBoard() const
{
  return this->pdb_.get();
}

MarchRobot::~MarchRobot()
{
  stopEtherCAT();
}

const urdf::Model& MarchRobot::getUrdf() const
{
  return this->urdf_;
}

}  // namespace march
