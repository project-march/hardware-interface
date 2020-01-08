// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_PDOMAP_H
#define MARCH_HARDWARE_PDOMAP_H

#include <string>
#include <utility>
#include <vector>
#include <map>
#include <unordered_map>

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatSDO.h>

namespace march4cpp
{
/** Store IMC data as a struct to prevent data overlap.*/
struct IMCObject
{
  int address;  // in IMC memory (see IMC manual)
  int length;   // bits (see IMC manual)

  explicit IMCObject(int _address, int _length) : address(_address), length(_length)
  {
  }

  IMCObject(){};
};

/** The data direction to which the PDO is specified is restricted to master in slave out and slave out master in.*/
enum class dataDirection
{
  miso,
  mosi
};

/** All the available IMC object names divided over the PDO maps. make sure to also add it to PDOmap constructor.*/
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
  /** Initiate all the entered IMC objects to prepare the PDO.*/
  void add_object(IMCObjectName object_name);

  std::map<IMCObjectName, int> map(int slaveIndex, dataDirection direction);

  static std::unordered_map<IMCObjectName, IMCObject> all_objects;

private:
  /** Used to sort the objects in the all_objects according to data length.*/
  void sort_PDO_objects();

  /** Configures the PDO in the IMC using the given base register address and sync manager address.
   * @return map of the IMC PDO object name in combination with the byte-offset in the PDO register */
  std::map<IMCObjectName, int> configurePDO(int slaveIndex, int baseRegister, int baseSyncManager);

  /** Combine the address(hex), sub-index(hex) and length(hex). *
   * @return combination of both address, sub-index and length (Example control word: 60400010h.) */
  static uint32_t combineAddressLength(uint16_t address, uint16_t length);

  std::unordered_map<IMCObjectName, IMCObject> PDO_objects;
  std::vector<std::pair<IMCObjectName, IMCObject>> sorted_PDO_objects;

  const int bits_per_register = 64;           // Maximum amount of bits that can be constructed in one PDO message.
  const int nr_of_regs = 4;                   // Amount of registers available.
  const int object_sizes[3] = { 32, 16, 8 };  // Available sizes.
};
}  // namespace march4cpp

#endif  // MARCH_HARDWARE_PDOMAP_H
