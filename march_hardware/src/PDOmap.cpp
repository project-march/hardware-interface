// Copyright 2019 Project March.
#include <march_hardware/PDOmap.h>

#include <map>
#include <utility>

namespace march4cpp
{
PDOmap::PDOmap()
{
  this->allObjects[IMCObjectName::StatusWord] = IMCObject(0x6041, 16);
  this->allObjects[IMCObjectName::ActualPosition] = IMCObject(0x6064, 32);
  this->allObjects[IMCObjectName::MotionErrorRegister] = IMCObject(0x2000, 16);
  this->allObjects[IMCObjectName::DetailedErrorRegister] = IMCObject(0x2002, 16);
  this->allObjects[IMCObjectName::DCLinkVoltage] = IMCObject(0x2055, 16);
  this->allObjects[IMCObjectName::DriveTemperature] = IMCObject(0x2058, 16);
  this->allObjects[IMCObjectName::ActualTorque] = IMCObject(0x6077, 16);
  this->allObjects[IMCObjectName::CurrentLimit] = IMCObject(0x207F, 16);
  this->allObjects[IMCObjectName::MotorPosition] = IMCObject(0x2088, 32);
  this->allObjects[IMCObjectName::ControlWord] = IMCObject(0x6040, 16);
  this->allObjects[IMCObjectName::TargetPosition] = IMCObject(0x607A, 32);
  this->allObjects[IMCObjectName::TargetTorque] = IMCObject(0x6071, 16);
  this->allObjects[IMCObjectName::QuickStopDeceleration] = IMCObject(0x6085, 32);
  this->allObjects[IMCObjectName::QuickStopOption] = IMCObject(0x605A, 16);
}

void PDOmap::addObject(IMCObjectName objectName)
{
  if (this->allObjects.count(objectName) != 1)
  {
    ROS_WARN("IMC object does not exist (yet), or multiple exist");
    return;
  }
  else if (this->PDOObjects.count(objectName) != 0)
  {
    ROS_WARN("IMC object is already added to PDO map");
    return;
  }
  else
  {
    this->PDOObjects[objectName] = this->allObjects[objectName];
  }
}

std::map<enum IMCObjectName, int> PDOmap::map(int slaveIndex, enum dataDirection direction)
{
  this->sortPDOObjects();

  if (direction == dataDirection::miso)
  {
    ROS_INFO("Mapping miso");
    return configurePDO(slaveIndex, 0x1A00, 0x1C13);
  }
  else if (direction == dataDirection::mosi)
  {
    ROS_INFO("Mapping mosi");
    return configurePDO(slaveIndex, 0x1600, 0x1C12);
  }
  else
  {
    throw std::runtime_error("Invalid dataDirection argument in PDO mapping");
  }
}

std::map<IMCObjectName, int> PDOmap::configurePDO(int slaveIndex, int baseRegister, int baseSyncManager)
{
  int counter = 1;
  int currentReg = baseRegister;
  int sizeLeft = this->bitsPerReg;

  sdo_bit8(slaveIndex, baseRegister, 0, 0);

  for (const auto& nextObject : sortedPDOObjects)
  {
    sizeLeft -= nextObject.second.length;
    if (sizeLeft < 0)
    {
      // PDO is filled so it can be enabled again
      sdo_bit8(slaveIndex, currentReg, 0, counter - 1);

      // Update the sync manager with the just configured PDO
      sdo_bit8(slaveIndex, baseSyncManager, 0, 0);
      int currentPDONr = (currentReg - baseRegister) + 1;
      sdo_bit16(slaveIndex, baseSyncManager, currentPDONr, currentReg);

      // Move to the next PDO register by incrementing with one
      currentReg++;
      if (currentReg > (baseRegister + nrofRegs))
      {
        ROS_ERROR("Amount of registers was overwritten, amount of parameters does not fit in the PDO messages.");
      }

      sizeLeft = this->bitsPerReg - nextObject.second.length;
      counter = 1;

      sdo_bit8(slaveIndex, currentReg, 0, 0);
    }

    int byteOffset = (bitsPerReg - (sizeLeft + nextObject.second.length)) / 8;
    this->byteOffsets[nextObject.first] = byteOffset;

    sdo_bit32(slaveIndex, currentReg, counter,
              this->combineAddressLength(nextObject.second.address, nextObject.second.length));
    counter++;
  }

  // Make sure the last PDO and sync manager are activated
  sdo_bit8(slaveIndex, currentReg, 0, counter - 1);
  sdo_bit8(slaveIndex, baseSyncManager, 0, 0);
  int currentPDONr = (currentReg - baseRegister) + 1;
  sdo_bit16(slaveIndex, baseSyncManager, currentPDONr, currentReg);

  // Explicitly disable PDO registers which are not used
  currentReg++;
  if (currentReg <= (baseRegister + nrofRegs))
  {
    for (int unusedRegister = currentReg; unusedRegister < baseRegister + this->nrofRegs; unusedRegister++)
    {
      sdo_bit8(slaveIndex, unusedRegister, 0, 0);
    }
  }

  // Active the sync manager again
  int totalAmountPDO = (currentReg - baseRegister);
  sdo_bit8(slaveIndex, baseSyncManager, 0, totalAmountPDO);

  return this->byteOffsets;
}

void PDOmap::sortPDOObjects()
{
  int totalBits = 0;
  for (int objectSize : this->objectSizes)
  {
    for (const auto& object : PDOObjects)
    {
      if (object.second.length == objectSize)
      {
        this->sortedPDOObjects.emplace_back(object);
        totalBits += objectSize;
      }
    }
  }

  if (totalBits > this->nrofRegs * this->bitsPerReg)
  {
    ROS_FATAL("Too many objects in PDO Map (total bits %d, only %d allowed)", totalBits,
              this->nrofRegs * this->bitsPerReg);
    throw std::exception();
  }
}

uint32_t PDOmap::combineAddressLength(uint16_t address, uint16_t length)
{
  uint32_t MSword = ((address & 0xFFFF) << 16);  // Shift 16 bits left for most significant word
  uint32_t LSword = (length & 0xFFFF);
  return (MSword | LSword);
}

}  // namespace march4cpp
