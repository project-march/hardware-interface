// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "march_hardware/Joint.h"
#include "march_hardware/TemperatureGES.h"
#include "march_hardware/IMotionCube.h"
#include "mocks/MockEncoderAbsolute.cpp"

using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::Return;

class TestEncoderAbsolute : public ::testing::Test
{
protected:
  const float angle = 42;
};

TEST_F(TestEncoderAbsolute, EncoderAbsolute)
{
  MockEncoderAbsolute mockEncoderAbsolute;

  EXPECT_CALL(mockEncoderAbsolute, getAngleDeg()).Times(AtLeast(1));
  ON_CALL(mockEncoderAbsolute, getAngleDeg()).WillByDefault(Return(angle));

  ASSERT_EQ(angle, mockEncoderAbsolute.getAngleDeg());
}
