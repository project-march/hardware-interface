// Copyright 2019 Project March.
#include <march_hardware_builder/hardware_config_exceptions.h>
#include <march_hardware_builder/hardware_builder.h>

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>

#include <march_hardware/EncoderIncremental.h>

class TestEncoderIncrementalBuilder : public ::testing::Test
{
protected:
  std::string base_path;

  void SetUp() override
  {
    this->base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/encoder");
  }

  YAML::Node loadTestYaml(const std::string& relative_path)
  {
    return YAML::LoadFile(this->base_path.append(relative_path));
  }
};

TEST_F(TestEncoderIncrementalBuilder, ValidEncoderIncremental)
{
  YAML::Node config = this->loadTestYaml("/encoder_incremental_correct.yaml");

  march::EncoderIncremental expected = march::EncoderIncremental(12);
  march::EncoderIncremental created = HardwareBuilder::createEncoderIncremental(config);
  ASSERT_EQ(expected, created);
}

TEST_F(TestEncoderIncrementalBuilder, NoResolution)
{
  YAML::Node config;

  ASSERT_THROW(HardwareBuilder::createEncoderIncremental(config), MissingKeyException);
}
