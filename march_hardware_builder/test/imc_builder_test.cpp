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

class IMotionCubeTest : public ::testing::Test
{
protected:
  std::string base_path;

  void SetUp() override
  {
    base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/imotioncube");
  }

  std::string fullPath(const std::string& relativePath)
  {
    return this->base_path.append(relativePath);
  }
};

TEST_F(IMotionCubeTest, ValidIMotionCubeHip)
{
  std::string fullPath = this->fullPath("/imotioncube_correct_1.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  march::IMotionCube createdIMotionCube = HardwareBuilder::createIMotionCube(iMotionCubeConfig);

  march::EncoderAbsolute actualEncoderAbsolute = march::EncoderAbsolute(16, 22134, 43436, 24515, 0.05);
  march::EncoderIncremental actualEncoderIncremental = march::EncoderIncremental(12);
  march::IMotionCube actualIMotionCube = march::IMotionCube(2, actualEncoderIncremental, actualEncoderAbsolute);

  ASSERT_EQ(actualIMotionCube, createdIMotionCube);
}

TEST_F(IMotionCubeTest, ValidIMotionCubeAnkle)
{
  std::string fullPath = this->fullPath("/imotioncube_correct_2.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  march::IMotionCube createdIMotionCube = HardwareBuilder::createIMotionCube(iMotionCubeConfig);

  march::EncoderAbsolute actualEncoderAbsolute = march::EncoderAbsolute(12, 1, 1000, 300, 0.01);
  march::EncoderIncremental actualEncoderIncremental = march::EncoderIncremental(13);
  march::IMotionCube actualIMotionCube = march::IMotionCube(10, actualEncoderIncremental, actualEncoderAbsolute);

  ASSERT_EQ(actualIMotionCube, createdIMotionCube);
}

TEST_F(IMotionCubeTest, IncorrectEncoderAbsolute)
{
  std::string fullPath = this->fullPath("/imotioncube_incorrect_encoder_absolute.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

TEST_F(IMotionCubeTest, IncorrectEncoderIncremental)
{
  std::string fullPath = this->fullPath("/imotioncube_incorrect_encoder_incremental.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

TEST_F(IMotionCubeTest, NoEncoderAbsolute)
{
  std::string fullPath = this->fullPath("/imotioncube_no_encoder_absolute.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

TEST_F(IMotionCubeTest, NoEncoderIncremental)
{
  std::string fullPath = this->fullPath("/imotioncube_no_encoder_incremental.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

TEST_F(IMotionCubeTest, NoSlaveIndex)
{
  std::string fullPath = this->fullPath("/imotioncube_no_slave_index.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig), MissingKeyException);
}
