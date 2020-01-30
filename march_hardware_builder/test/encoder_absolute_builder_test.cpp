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

class EncoderAbsoluteTest : public ::testing::Test
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

TEST_F(EncoderAbsoluteTest, ValidEncoderHip)
{
  std::string fullPath = this->fullPath("/encoder_absolute_correct_1.yaml");
  YAML::Node encoderAbsoluteConfig = YAML::LoadFile(fullPath);

  march::EncoderAbsolute actualEncoderAbsolute = march::EncoderAbsolute(16, 22134, 43436, 24515, 0.05);
  march::EncoderAbsolute createdEncoderAbsolute = HardwareBuilder::createAbsoluteEncoder(encoderAbsoluteConfig);
  ASSERT_EQ(actualEncoderAbsolute, createdEncoderAbsolute);
}

TEST_F(EncoderAbsoluteTest, ValidEncoderAnkle)
{
  std::string fullPath = this->fullPath("/encoder_absolute_correct_2.yaml");
  YAML::Node encoderAbsoluteConfig = YAML::LoadFile(fullPath);

  march::EncoderAbsolute actualEncoderAbsolute = march::EncoderAbsolute(12, 1086, 1490, 1301, 0.005);

  march::EncoderAbsolute createdEncoderAbsolute = HardwareBuilder::createAbsoluteEncoder(encoderAbsoluteConfig);
  ASSERT_EQ(actualEncoderAbsolute, createdEncoderAbsolute);
}

TEST_F(EncoderAbsoluteTest, NoResolution)
{
  std::string fullPath = this->fullPath("/encoder_absolute_no_resolution.yaml");
  YAML::Node encoderAbsoluteConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(encoderAbsoluteConfig), MissingKeyException);
}

TEST_F(EncoderAbsoluteTest, NoMinPosition)
{
  std::string fullPath = this->fullPath("/encoder_absolute_no_min_position.yaml");
  YAML::Node encoderAbsoluteConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(encoderAbsoluteConfig), MissingKeyException);
}

TEST_F(EncoderAbsoluteTest, NoMaxPosition)
{
  std::string fullPath = this->fullPath("/encoder_absolute_no_max_position.yaml");
  YAML::Node encoderAbsoluteConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(encoderAbsoluteConfig), MissingKeyException);
}

TEST_F(EncoderAbsoluteTest, NoZeroPosition)
{
  std::string fullPath = this->fullPath("/encoder_absolute_no_zero_position.yaml");
  YAML::Node encoderAbsoluteConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(encoderAbsoluteConfig), MissingKeyException);
}

TEST_F(EncoderAbsoluteTest, NoSafetyMargin)
{
  std::string fullPath = this->fullPath("/encoder_absolute_no_safety_margin.yaml");
  YAML::Node encoderAbsoluteConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(encoderAbsoluteConfig), MissingKeyException);
}
