// Copyright 2018 Project March.
#include "march_hardware/Encoder.h"

#include <tuple>

#include <gtest/gtest.h>
#include <march_hardware/error/hardware_exception.h>

class TestEncoder : public testing::Test
{
protected:
  const int32_t lower_limit = 2053;
  const int32_t upper_limit = 45617;
  march::Encoder encoder =
      march::Encoder(17, lower_limit, upper_limit, -0.34906585, 1.745329252, -0.296705973, 1.692969374);
};

class TestEncoderParameterizedLimits : public TestEncoder, public testing::WithParamInterface<std::tuple<int32_t, bool>>
{
};

class TestEncoderParameterizedSoftLimits : public TestEncoder,
                                           public testing::WithParamInterface<std::tuple<int32_t, bool>>
{
};

class TestEncoderParameterizedValidTarget : public TestEncoder,
                                            public testing::WithParamInterface<std::tuple<int32_t, int32_t, bool>>
{
};

TEST_F(TestEncoder, CorrectLimits)
{
  ASSERT_EQ(this->encoder.getLowerHardLimitIU(), this->lower_limit);
  ASSERT_EQ(this->encoder.getUpperHardLimitIU(), this->upper_limit);
}

TEST_F(TestEncoder, CorrectSoftLimits)
{
  ASSERT_EQ(this->encoder.getLowerSoftLimitIU(), 3144);
  ASSERT_EQ(this->encoder.getUpperSoftLimitIU(), 44650);
}

TEST(TestEncoderConstructor, ResolutionBelowRange)
{
  ASSERT_THROW(march::Encoder(0, 0, 0, 0, 0, 0, 0), march::error::HardwareException);
}

TEST(TestEncoderConstructor, ResolutionAboveRange)
{
  ASSERT_THROW(march::Encoder(50, 0, 0, 0, 0, 0, 0), march::error::HardwareException);
}

TEST(TestEncoderConstructor, DifferentIuPerRad)
{
  ASSERT_THROW(march::Encoder(12, 2, 1, 1.0, 2.0, 1.0, 2.0), march::error::HardwareException);
}

TEST(TestEncoderConstructor, InvalidSoftLimits)
{
  ASSERT_THROW(march::Encoder(17, 2053, 45617, -0.34906585, 1.745329252, 0.4, 0.1), march::error::HardwareException);
}

TEST(TestEncoderConstructor, InvalidLowerSoftLimit)
{
  ASSERT_THROW(march::Encoder(17, 2053, 45617, -0.34906585, 1.745329252, -0.4, 1), march::error::HardwareException);
}

TEST(TestEncoderConstructor, InvalidUpperSoftLimit)
{
  ASSERT_THROW(march::Encoder(17, 2053, 45617, -0.34906585, 1.745329252, -0.3, 2.0), march::error::HardwareException);
}

INSTANTIATE_TEST_CASE_P(ParameterizedLimits, TestEncoderParameterizedLimits,
                        testing::Values(std::make_tuple(2052, false), std::make_tuple(2053, false),
                                        std::make_tuple(2054, true), std::make_tuple(45616, true),
                                        std::make_tuple(45617, false), std::make_tuple(45618, false),
                                        std::make_tuple(3000, true), std::make_tuple(0, false),
                                        std::make_tuple(60001, false)));

TEST_P(TestEncoderParameterizedLimits, IsWithinHardLimits)
{
  const int32_t iu = std::get<0>(this->GetParam());
  const bool expected = std::get<1>(this->GetParam());
  ASSERT_EQ(this->encoder.isWithinHardLimitsIU(iu), expected);
}

INSTANTIATE_TEST_CASE_P(ParameterizedSoftLimits, TestEncoderParameterizedSoftLimits,
                        testing::Values(std::make_tuple(3143, false), std::make_tuple(3144, false),
                                        std::make_tuple(3145, true), std::make_tuple(44649, true),
                                        std::make_tuple(44650, false), std::make_tuple(44651, false),
                                        std::make_tuple(4500, true), std::make_tuple(0, false),
                                        std::make_tuple(60001, false)));

TEST_P(TestEncoderParameterizedSoftLimits, IsWithinSoftLimits)
{
  const int32_t iu = std::get<0>(this->GetParam());
  const bool expected = std::get<1>(this->GetParam());
  ASSERT_EQ(this->encoder.isWithinSoftLimitsIU(iu), expected);
}

INSTANTIATE_TEST_CASE_P(ParameterizedValidTarget, TestEncoderParameterizedValidTarget,
                        testing::Values(std::make_tuple(4000, 4500, true), std::make_tuple(4000, 2000, false),
                                        std::make_tuple(46000, 40000, true), std::make_tuple(46000, 45900, true),
                                        std::make_tuple(44560, 44560, true), std::make_tuple(46000, 46001, false),
                                        std::make_tuple(46000, 2000, false), std::make_tuple(3000, 5000, true),
                                        std::make_tuple(3000, 3001, true), std::make_tuple(3144, 3144, true),
                                        std::make_tuple(0, -1, false), std::make_tuple(0, 60000, false)));

TEST_P(TestEncoderParameterizedValidTarget, IsValidTargetIU)
{
  const int32_t current_iu = std::get<0>(this->GetParam());
  const int32_t target_iu = std::get<1>(this->GetParam());
  const bool expected = std::get<2>(this->GetParam());
  ASSERT_EQ(this->encoder.isValidTargetIU(current_iu, target_iu), expected);
}
