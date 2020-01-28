// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_IMOTIONCUBE_H
#define MARCH_HARDWARE_IMOTIONCUBE_H

#include <unordered_map>
#include <string>

#include <march_hardware/ActuationMode.h>
#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Slave.h>
#include <march_hardware/EncoderIncremental.h>
#include <march_hardware/EncoderAbsolute.h>
#include <march_hardware/PDOmap.h>
#include <march_hardware/IMotionCubeState.h>
#include <march_hardware/IMotionCubeTargetState.h>

namespace march
{
class IMotionCube : public Slave
{
private:
  EncoderIncremental encoderIncremental;
  EncoderAbsolute encoderAbsolute;
  ActuationMode actuationMode;

  void actuateIU(int iu);

  std::unordered_map<IMCObjectName, int> misoByteOffsets;
  std::unordered_map<IMCObjectName, int> mosiByteOffsets;
  void mapMisoPDOs();
  void mapMosiPDOs();
  void validateMisoPDOs();
  void validateMosiPDOs();
  void writeInitialSettings(uint8 ecatCycleTime);

  bool get_bit(uint16 value, int index);

public:
  explicit IMotionCube(int slaveIndex, EncoderIncremental encoderIncremental, EncoderAbsolute encoderAbsolute);

  IMotionCube()
  {
    slaveIndex = -1;
  }

  ~IMotionCube() = default;

  void writeInitialSDOs(int ecatCycleTime) override;

  float getAngleRadAbsolute();
  float getAngleRadIncremental();
  float getTorque();
  int getAngleIUabsolute();
  int getAngleIUincremental();

  uint16 getStatusWord();
  uint16 getMotionError();
  uint16 getDetailedError();

  ActuationMode getActuationMode() const;

  float getMotorCurrent();
  float getMotorVoltage();

  void setControlWord(uint16 controlWord);

  void actuateRad(float targetRad);
  void actuateTorque(int targetTorque);

  std::string parseStatusWord(uint16 statusWord);
  IMCState getState(uint16 statusWord);
  std::string parseMotionError(uint16 motionError);
  std::string parseDetailedError(uint16 detailedError);

  bool goToOperationEnabled();
  bool resetIMotionCube();

  void setActuationMode(ActuationMode mode);

  /** @brief Override comparison operator */
  friend bool operator==(const IMotionCube& lhs, const IMotionCube& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.encoderIncremental == rhs.encoderIncremental &&
           lhs.encoderAbsolute == rhs.encoderAbsolute;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const IMotionCube& iMotionCube)
  {
    return os << "slaveIndex: " << iMotionCube.slaveIndex << ", "
              << "encoderIncremental: " << iMotionCube.encoderIncremental << ", "
              << "encoderAbsolute: " << iMotionCube.encoderAbsolute;
  }
  bool goToTargetState(march::IMotionCubeTargetState targetState);
};

}  // namespace march
#endif  // MARCH_HARDWARE_IMOTIONCUBE_H
