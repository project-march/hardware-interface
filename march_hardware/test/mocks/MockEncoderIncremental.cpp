#include "march_hardware/encoder/EncoderIncremental.h"

#include <gmock/gmock.h>

class MockEncoderIncremental : public march::EncoderIncremental
{
public:
  MockEncoderIncremental() : EncoderIncremental(10)
  {
  }

  MOCK_METHOD0(getAngleIU, int32_t());
  MOCK_METHOD0(getAngleRad, double());
};
