// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MARCH_ROBOT_H
#define MARCH_HARDWARE_MARCH_ROBOT_H
#include "march_hardware/ethercat/ethercat_master.h"
#include "march_hardware/joint.h"
#include "march_hardware/power/power_distribution_board.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

namespace march
{
class MarchRobot
{
private:
  ::std::vector<Joint> jointList;
  urdf::Model urdf_;
  EthercatMaster ethercatMaster;
  std::shared_ptr<PowerDistributionBoard> pdb_;

public:
  using iterator = std::vector<Joint>::iterator;

  MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf, ::std::string ifName,
             std::vector<std::shared_ptr<Slave>> slave_list, int ecatCycleTime, int ecatSlaveTimeout);

  MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
             std::shared_ptr<PowerDistributionBoard> powerDistributionBoard, ::std::string ifName,
             std::vector<std::shared_ptr<Slave>> slave_list, int ecatCycleTime, int ecatSlaveTimeout);

  ~MarchRobot();

  /* Delete copy constructor/assignment since the ethercat master can not be copied */
  MarchRobot(const MarchRobot&) = delete;
  MarchRobot& operator=(const MarchRobot&) = delete;

  /* Delete move constructor/assignment since atomic bool cannot be moved */
  MarchRobot(MarchRobot&&) = delete;
  MarchRobot& operator=(MarchRobot&&) = delete;

  void resetMotorControllers();

  void startEtherCAT(bool reset_motor_controllers);

  void stopEtherCAT();

  int getMaxSlaveIndex();

  bool isEthercatOperational();

  std::exception_ptr getLastEthercatException() const noexcept;

  void waitForPdo();

  int getEthercatCycleTime() const;

  Joint& getJoint(::std::string jointName);

  Joint& getJoint(size_t index);

  size_t size() const;

  iterator begin();
  iterator end();

  bool hasPowerDistributionboard() const;
  PowerDistributionBoard* getPowerDistributionBoard() const;

  const urdf::Model& getUrdf() const;
};
}  // namespace march
#endif  // MARCH_HARDWARE_MARCH_ROBOT_H
