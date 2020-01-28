#include "gmock/gmock.h"  // Brings in Google Mock.
#include "march_hardware/EncoderIncremental.h"

class MockEncoderIncremental : public march::EncoderIncremental
{
public:
  MOCK_METHOD0(getAngleDeg, float());
  MOCK_METHOD0(getAngleRad, float());
  MOCK_METHOD0(getAngle, float());
};