// Copyright 2019 Project March.

#ifndef MARCH4CPP__IMOTIONCUBE_H
#define MARCH4CPP__IMOTIONCUBE_H

#include <map>
#include <string>

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Slave.h>
#include <march_hardware/Encoder.h>
#include <march_hardware/PDOmap.h>
#include <march_hardware/ActuationMode.h>
#include <march_hardware/IMotionCubeState.h>

namespace march4cpp
{
class IMotionCube : public Slave
{
private:
  Encoder encoder;
  void actuateIU(int iu);

  std::map<IMCObjectName, int> misoByteOffsets;
  std::map<IMCObjectName, int> mosiByteOffsets;
  void mapMisoPDOs();
  void mapMosiPDOs();
  void validateMisoPDOs();
  void validateMosiPDOs();
  void writeInitialSettings(uint8 ecatCycleTime, uint8_t modeofOp);
  bool get_bit(uint16 value, int index);
  ActuationMode actuationMode;

public:
  explicit IMotionCube(int slaveIndex, Encoder encoder);
  explicit IMotionCube(int slaveIndex, Encoder encoder, ActuationMode actuationmode);

  IMotionCube()
  {
    slaveIndex = -1;
  }

  ~IMotionCube() = default;

  void writeInitialSDOs(int ecatCycleTime, ActuationMode actuationmode);

  float getAngleRad();
  float getTorque();
  int getAngleIU();

  uint16 getStatusWord();
  uint16 getMotionError();
  uint16 getDetailedError();

  void setControlWord(uint16 controlWord);

  void actuateRad(float targetRad);
  void actuateTorque(int targetTorque);

  void actuateRadFixedSpeed(float targetRad, float radPerSec);

  std::string parseStatusWord(uint16 statusWord);
  IMCState getState(uint16 statusWord);
  std::string parseMotionError(uint16 motionError);
  std::string parseDetailedError(uint16 detailedError);

  bool goToOperationEnabled();
  bool resetIMotionCube();

  /** @brief Override comparison operator */
  friend bool operator==(const IMotionCube& lhs, const IMotionCube& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.encoder == rhs.encoder;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const IMotionCube& iMotionCube)
  {
    return os << "slaveIndex: " << iMotionCube.slaveIndex << ", "
              << "encoder: " << iMotionCube.encoder;
  }
};

}  // namespace march4cpp
#endif  // MARCH4CPP__IMOTIONCUBE_H
