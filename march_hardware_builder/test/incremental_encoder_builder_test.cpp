// Copyright 2019 Project March.
#include <march_hardware_builder/hardware_config_exceptions.h>
#include <march_hardware_builder/hardware_builder.h>

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>

#include <march_hardware/encoder/IncrementalEncoder.h>

class TestIncrementalEncoderBuilder : public ::testing::Test
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

TEST_F(TestIncrementalEncoderBuilder, ValidIncrementalEncoder)
{
  YAML::Node config = this->loadTestYaml("/incremental_encoder_correct.yaml");

  march::IncrementalEncoder expected = march::IncrementalEncoder(12);
  march::IncrementalEncoder created = HardwareBuilder::createIncrementalEncoder(config);
  ASSERT_EQ(expected, created);
}

TEST_F(TestIncrementalEncoderBuilder, NoResolution)
{
  YAML::Node config;

  ASSERT_THROW(HardwareBuilder::createIncrementalEncoder(config), MissingKeyException);
}
