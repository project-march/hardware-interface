// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <march_hardware/encoder/EncoderAbsolute.h>
#include <march_hardware/encoder/EncoderIncremental.h>
#include <march_hardware/IMotionCube.h>

class IMotionCubeTest : public ::testing::Test
{
protected:
  std::string base_path;
  urdf::JointSharedPtr joint;

  void SetUp() override
  {
    this->base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/imotioncube");
    this->joint = std::make_shared<urdf::Joint>();
    this->joint->limits = std::make_shared<urdf::JointLimits>();
    this->joint->safety = std::make_shared<urdf::JointSafety>();
  }

  YAML::Node loadTestYaml(const std::string& relative_path)
  {
    return YAML::LoadFile(this->base_path.append(relative_path));
  }
};

TEST_F(IMotionCubeTest, ValidIMotionCubeHip)
{
  YAML::Node config = this->loadTestYaml("/imotioncube_correct.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  march::IMotionCube created = HardwareBuilder::createIMotionCube(config, march::ActuationMode::unknown, this->joint);

  march::EncoderAbsolute encoder_absolute =
      march::EncoderAbsolute(16, 22134, 43436, this->joint->limits->lower, this->joint->limits->upper,
                             this->joint->safety->soft_lower_limit, this->joint->safety->soft_upper_limit);
  march::EncoderIncremental encoder_incremental = march::EncoderIncremental(12);
  march::IMotionCube expected =
      march::IMotionCube(2, encoder_absolute, encoder_incremental, march::ActuationMode::unknown);

  ASSERT_EQ(expected, created);
}

TEST_F(IMotionCubeTest, NoEncoderAbsolute)
{
  YAML::Node iMotionCubeConfig = this->loadTestYaml("/imotioncube_no_encoder_absolute.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig, march::ActuationMode::unknown, this->joint),
               MissingKeyException);
}

TEST_F(IMotionCubeTest, NoEncoderIncremental)
{
  YAML::Node iMotionCubeConfig = this->loadTestYaml("/imotioncube_no_encoder_incremental.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig, march::ActuationMode::unknown, this->joint),
               MissingKeyException);
}

TEST_F(IMotionCubeTest, NoSlaveIndex)
{
  YAML::Node iMotionCubeConfig = this->loadTestYaml("/imotioncube_no_slave_index.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig, march::ActuationMode::unknown, this->joint),
               MissingKeyException);
}
