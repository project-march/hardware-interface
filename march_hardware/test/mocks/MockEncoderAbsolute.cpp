#include "march_hardware/EncoderAbsolute.h"

#include <gmock/gmock.h>

class MockEncoderAbsolute : public march::EncoderAbsolute
{
public:
  MockEncoderAbsolute() : EncoderAbsolute(10, 0, 162, 0, 1, 0.1, 0.9)
  {
  }

  MOCK_METHOD0(getAngleIU, int32_t());
  MOCK_METHOD0(getAngleRad, double());
};
