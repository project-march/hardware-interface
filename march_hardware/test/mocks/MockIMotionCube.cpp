#include "MockEncoderAbsolute.cpp"
#include "MockEncoderIncremental.cpp"

#include "march_hardware/IMotionCube.h"

#include <gmock/gmock.h>

class MockIMotionCube : public march::IMotionCube
{
public:
  MockIMotionCube() : IMotionCube(1, MockEncoderAbsolute(), MockEncoderIncremental(), march::ActuationMode::unknown)
  {
  }
  MOCK_METHOD0(getAngle, float());
};
