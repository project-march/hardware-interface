// Copyright 2020 Project March.
#include "march_hardware/encoder/Encoder.h"
#include "march_hardware/error/hardware_exception.h"

#include <cmath>

#include <gtest/gtest.h>

class TestEncoder : public testing::Test
{
protected:
  const size_t resolution = 12;
  march::Encoder encoder = march::Encoder(this->resolution);
};

TEST_F(TestEncoder, ResolutionBelowRange)
{
  ASSERT_THROW(march::Encoder(0), march::error::HardwareException);
}

TEST_F(TestEncoder, ResolutionAboveRange)
{
  ASSERT_THROW(march::Encoder(50), march::error::HardwareException);
}

TEST_F(TestEncoder, SetSlaveIndex)
{
  const int expected = 10;
  this->encoder.setSlaveIndex(expected);
  ASSERT_EQ(expected, this->encoder.getSlaveIndex());
}

TEST_F(TestEncoder, CorrectTotalPositions)
{
  ASSERT_EQ(std::pow(2, this->resolution), this->encoder.getTotalPositions());
}
