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

  float getAngleRad(uint8_t ActualPositionByteOffset);

  int getAngleIU(uint8_t ActualPositionByteOffset);

  float IUtoRad(int iu);
  int RadtoIU(float rad);

  void setSlaveIndex(int slaveIndex);

  int getSlaveIndex() const;
};
}  // namespace march

#endif  // MARCH_HARDWARE_ENCODER_H
