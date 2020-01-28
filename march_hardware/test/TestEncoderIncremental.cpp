// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "march_hardware/Joint.h"
#include "march_hardware/TemperatureGES.h"
#include "march_hardware/IMotionCube.h"
#include "mocks/MockEncoderIncremental.cpp"

using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::Return;

class TestEncoderIncremental : public ::testing::Test
{
protected:
  const float angle = 42;
};

TEST_F(TestEncoderIncremental, EncoderIncremental)
{
  MockEncoderIncremental mockEncoderIncremental;

  EXPECT_CALL(mockEncoderIncremental, getAngleDeg()).Times(AtLeast(1));
  ON_CALL(mockEncoderIncremental, getAngleDeg()).WillByDefault(Return(angle));

  ASSERT_EQ(angle, mockEncoderIncremental.getAngleDeg());
}
