// Copyright 2019 Project March.
#include <string>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/hardware_config_exceptions.h>
#include <march_hardware_builder/hardware_builder.h>

using ::testing::AtLeast;
using ::testing::Return;

class EncoderIncrementalTest : public ::testing::Test
{
protected:
  std::string base_path;

  void SetUp() override
  {
    base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/encoder");
  }

  std::string fullPath(const std::string& relativePath)
  {
    return this->base_path.append(relativePath);
  }
};

TEST_F(EncoderIncrementalTest, ValidEncoderIncrementalHip)
{
  std::string fullPath = this->fullPath("/encoder_incremental_correct_1.yaml");
  YAML::Node encoderIncrementalConfig = YAML::LoadFile(fullPath);

  march::EncoderIncremental actualEncoderIncremental = march::EncoderIncremental(12);
  march::EncoderIncremental createdEncoderIncremental =
      HardwareBuilder::createEncoderIncremental(encoderIncrementalConfig);
  ASSERT_EQ(actualEncoderIncremental, createdEncoderIncremental);
}

TEST_F(EncoderIncrementalTest, ValidEncoderIncrementalAnkle)
{
  std::string fullPath = this->fullPath("/encoder_incremental_correct_2.yaml");
  YAML::Node encoderIncrementalConfig = YAML::LoadFile(fullPath);

  march::EncoderIncremental actualEncoderIncremental = march::EncoderIncremental(13);

  march::EncoderIncremental createdEncoderIncremental =
      HardwareBuilder::createEncoderIncremental(encoderIncrementalConfig);
  ASSERT_EQ(actualEncoderIncremental, createdEncoderIncremental);
}

TEST_F(EncoderIncrementalTest, NoResolution)
{
  std::string fullPath = this->fullPath("/encoder_incremental_no_resolution.yaml");
  YAML::Node encoderIncrementalConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createEncoderIncremental(encoderIncrementalConfig), MissingKeyException);
}
