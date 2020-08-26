// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <march_hardware/encoder/absolute_encoder.h>
#include <march_hardware/encoder/incremental_encoder.h>
#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/motor_controller/imotioncube/imotioncube.h>

class JointBuilderTest : public ::testing::Test
{
protected:
  std::string base_path;
  urdf::JointSharedPtr joint;
  march::PdoInterfacePtr pdo_interface;
  march::SdoInterfacePtr sdo_interface;
  AllowedRobot robot;
  HardwareBuilder builder = HardwareBuilder(robot);

  void SetUp() override
  {
    this->base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/joint");
    this->joint = std::make_shared<urdf::Joint>();
    this->joint->limits = std::make_shared<urdf::JointLimits>();
    this->joint->safety = std::make_shared<urdf::JointSafety>();
    this->pdo_interface = march::PdoInterfaceImpl::create();
    this->sdo_interface = march::SdoInterfaceImpl::create();
  }

  YAML::Node loadTestYaml(const std::string& relative_path)
  {
    return YAML::LoadFile(this->base_path.append(relative_path));
  }
};

TEST_F(JointBuilderTest, ValidJointHip)
{
  YAML::Node config = this->loadTestYaml("/joint_correct.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  const std::string name = "test_joint_hip";
  march::Joint joint = this->builder.createJoint(config, name, this->joint, this->pdo_interface, this->sdo_interface);

  ASSERT_TRUE(joint.hasMotorController());
}

TEST_F(JointBuilderTest, ValidNotActuated)
{
  YAML::Node config = this->loadTestYaml("/joint_correct_not_actuated.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  march::Joint joint =
      this->builder.createJoint(config, "test_joint_hip", this->joint, this->pdo_interface, this->sdo_interface);

  ASSERT_TRUE(joint.hasMotorController());
}

TEST_F(JointBuilderTest, NoActuate)
{
  YAML::Node config = this->loadTestYaml("/joint_no_actuate.yaml");

  ASSERT_THROW(
      this->builder.createJoint(config, "test_joint_no_actuate", this->joint, this->pdo_interface, this->sdo_interface),
      MissingKeyException);
}

TEST_F(JointBuilderTest, NoIMotionCube)
{
  YAML::Node config = this->loadTestYaml("/joint_no_imotioncube.yaml");
  march::Joint joint = this->builder.createJoint(config, "test_joint_no_imotioncube", this->joint, this->pdo_interface,
                                                 this->sdo_interface);

  ASSERT_FALSE(joint.hasMotorController());
}

TEST_F(JointBuilderTest, NoTemperatureGES)
{
  YAML::Node config = this->loadTestYaml("/joint_no_temperature_ges.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 0.24;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 0.15;

  ASSERT_NO_THROW(this->builder.createJoint(config, "test_joint_no_temperature_ges", this->joint, this->pdo_interface,
                                            this->sdo_interface));
}

TEST_F(JointBuilderTest, ValidActuationMode)
{
  YAML::Node config = this->loadTestYaml("/joint_correct_position_mode.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  march::Joint joint =
      this->builder.createJoint(config, "test_joint_hip", this->joint, this->pdo_interface, this->sdo_interface);

  ASSERT_TRUE(joint.hasMotorController());
}

TEST_F(JointBuilderTest, EmptyJoint)
{
  YAML::Node config;
  ASSERT_THROW(
      this->builder.createJoint(config, "test_joint_empty", this->joint, this->pdo_interface, this->sdo_interface),
      MissingKeyException);
}

TEST_F(JointBuilderTest, NoUrdfJoint)
{
  YAML::Node config = this->loadTestYaml("/joint_correct.yaml");
  ASSERT_THROW(this->builder.createJoint(config, "test", nullptr, this->pdo_interface, this->sdo_interface),
               march::error::HardwareException);
}
