// Copyright 2019 Project March.
#ifndef MARCH4CPP__PDOMAP_H
#define MARCH4CPP__PDOMAP_H

#include <map>
#include <string>
#include <vector>
#include <queue>
#include <find>

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatSDO.h>

namespace march4cpp {

// If a new object is added to this enum, make sure to also add it to
// PDOmap::initAllObjects()!
enum class IMCObjectName {
  StatusWord,
  ActualPosition,
  MotionErrorRegister,
  DetailedErrorRegister,
  DCLinkVoltage,
  DriveTemperature,
  ActualTorque,
  CurrentLimit,
  MotorPosition,
  ControlWord,
  TargetPosition,
  TargetCurrent,
  QuickStopDeceleration,
  QuickStopOption
};

enum class dataDirection { miso, mosi };

struct IMCObject {
  int address; // in IMC memory (see IMC manual)
  int length;  // bits (see IMC manual)

  explicit IMCObject(int _address, int _length) {
    this->address = _address;
  }

  IMCObject() {}
};

class PDOmap {
public:
  // Constructor
  PDOmap();

  void addObject(IMCObjectName objectname, int reg);
  std::map<IMCObjectName, int> map(int slaveIndex, dataDirection direction);

private:
  void initIMCObjects();
  int mapObject(IMCObjectName objectName, int objectCount, int reg, int slaveIndex);
  uint32_t combineAddressLength(uint16_t address, uint16_t length);

  std::vector<std::pair<IMCObjectName, IMCObject>> PDOObjects;
  std::queue<IMCObjectName> mappedIMCObjects;
  std::queue<int> mappedIMCObjectRegisters;
  std::map<IMCObjectName, IMCObject> imcObjects;
  std::map<IMCObjectName, int> byteOffsets;

  const int bitsPerReg = 64;
  const int nrofRegs = 4;
  const int objectSizes[3] = {8, 16, 32};
};
} // namespace march4cpp

#endif
