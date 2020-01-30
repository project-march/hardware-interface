// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_H
#define MARCH_HARDWARE_ENCODER_H

namespace march
{
class Encoder
{
protected:
  int slaveIndex;
  int totalPositions;
public:
  Encoder()
    : slaveIndex(-1)
    , totalPositions(0)
  {
  }
  Encoder(int numberOfBits);

  int getAngleIU(uint8_t ActualPositionByteOffset);

  void setSlaveIndex(int slaveIndex);

  int getSlaveIndex() const;
};
}  // namespace march

#endif  // MARCH_HARDWARE_ENCODER_H
