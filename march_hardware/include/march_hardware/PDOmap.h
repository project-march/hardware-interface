// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_PDOMAP_H
#define MARCH_HARDWARE_PDOMAP_H

#include <string>
#include <utility>
#include <vector>
#include <map>

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatSDO.h>

namespace march4cpp
{

/**
 * Store IMC data as a struct to prevent data overlap
 */
struct IMCObject
{
  int address;  // in IMC memory (see IMC manual)
  int length;   // bits (see IMC manual)

  explicit IMCObject(int _address, int _length)
  {
    this->address = _address;
    this->length = _length;
  }

  IMCObject()
  {
  }
};

/**
 * The data direction to which the PDO is specified is restricted to master in slave out and slave out master in
 */
enum class dataDirection
{
  miso,
  mosi
};

/**
 * All the available IMC object names divided over the PDO maps
 * Note: If a new object is added to this enum, make sure to also add it to PDOmap::initAllObjects()!
 */
enum class IMCObjectName
{
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
  TargetTorque,
  QuickStopDeceleration,
  QuickStopOption
};

class PDOmap
{
public:
  PDOmap();
  void addObject(IMCObjectName objectName);
  std::map<IMCObjectName, int> map(int slaveIndex, dataDirection direction);

private:
  /**
 * Initiate all the entered IMC objects to prepare the PDO
 */
  void initAllObjects();

  /**
   * This function is used to sort the objects in the allObjects according to data length. This should optimize the
   * usage of storage
   */
  void sortPDOObjects();

  /**
 * Combine the address(hex), sub-index(hex) and length(hex). Example control word: 60400010h
 */
  uint32_t combineAddressLength(uint16_t address, uint16_t length);

  std::map<IMCObjectName, IMCObject> PDOObjects;
  std::map<IMCObjectName, IMCObject> allObjects;
  std::vector<std::pair<IMCObjectName, IMCObject>> sortedPDOObjects;
  std::map<IMCObjectName, int> byteOffsets;

  const int bitsPerReg = 64;                 // Maximum amount of bits that  constructed in one PDO message.
  const int nrofRegs = 4;                    // Amount of registers available.
  const int objectSizes[3] = { 32, 16, 8 };  // Available sizes.
};
}  // namespace march4cpp

#endif  // MARCH_HARDWARE_PDOMAP_H
