// Copyright 2019 Project March.
#include <march_hardware/PDOmap.h>

#include <map>
#include <utility>

namespace march4cpp
{
PDOmap::PDOmap()
{
  this->initAllObjects();
}

void PDOmap::initAllObjects()
{
  // Object(address, length);
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
  // etc...
  // If a new entry is added here, first add it to the enum (in the header file)!
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
  int reg{};
  int SMAddress{};

  if (direction == dataDirection::miso)
  {
    ROS_INFO("Mapping miso");
    reg = 0x1A00;
    SMAddress = 0x1C13;
  }
  else if (direction == dataDirection::mosi)
  {
    ROS_INFO("Mapping mosi");
    reg = 0x1600;
    SMAddress = 0x1C12;
  }
  else
  {
    ROS_ERROR("Invalid dataDirection argument");
  }

  int counter = 1;
  int currentReg = reg;
  int sizeLeft = this->bitsPerReg;

  sdo_bit8(slaveIndex, currentReg, 0, 0);

  ROS_INFO("current reg: 0x%4.4x", currentReg);
  for (const auto& nextObject : sortedPDOObjects)
  {
    sizeLeft -= nextObject.second.length;
    if (sizeLeft < 0)
    {
      // PDO is filled so it can be enabled again
      sdo_bit8(slaveIndex, currentReg, 0, counter - 1);

      // Change the sync manager accordingly
      sdo_bit8(slaveIndex, SMAddress, 0, 0);
      int currentPDONr = (currentReg - reg) + 1;
      sdo_bit16(slaveIndex, SMAddress, currentPDONr, currentReg);

      // Move to the next PDO
      currentReg++;
      if (currentReg > (reg + nrofRegs))
      {
        ROS_ERROR("Amount of registers was overwritten, amount of parameters does not fit in the PDO messages.");
      }

      sizeLeft = this->bitsPerReg - nextObject.second.length;
      counter = 1;

      sdo_bit8(slaveIndex, currentReg, 0, 0);
      ROS_INFO("current reg: 0x%4.4x", currentReg);
    }
    ROS_INFO("reg 0x%X, index %i, length %i,  0x%X", currentReg, nextObject.second.address, nextObject.second.length,
             this->combineAddressLength(nextObject.second.address, nextObject.second.length));

    int byteOffset = (bitsPerReg - (sizeLeft + nextObject.second.length)) / 8;
    this->byteOffsets[nextObject.first] = byteOffset;

    sdo_bit32(slaveIndex, currentReg, counter,
              this->combineAddressLength(nextObject.second.address, nextObject.second.length));
    counter++;
  }
  sdo_bit8(slaveIndex, currentReg, 0, counter - 1);

  sdo_bit8(slaveIndex, SMAddress, 0, 0);
  int currentPDONr = (currentReg - reg) + 1;
  sdo_bit16(slaveIndex, SMAddress, currentPDONr, currentReg);

  // explicitly disable PDO
  currentReg++;
  if (currentReg <= (reg + nrofRegs))
  {
    for (int unusedRegister = currentReg; unusedRegister < reg + this->nrofRegs; unusedRegister++)
    {
      int PDO = unusedRegister - reg;
      ROS_INFO("unused reg: 0x%4.4x, PDO: %i", unusedRegister, PDO);
      sdo_bit8(slaveIndex, unusedRegister, 0, 0);
    }
  }

  // Active the sync manager again
  int totalAmountPDO = (currentReg - reg);
  sdo_bit8(slaveIndex, SMAddress, 0, totalAmountPDO);
  ROS_INFO("total amount of PDO: %i", totalAmountPDO);

  return this->byteOffsets;
}

void PDOmap::sortPDOObjects()
{
  int totalBits = 0;
  for (int i = 0; i < (sizeof(this->objectSizes) / sizeof(this->objectSizes[0])); i++)
  {
    std::map<IMCObjectName, IMCObject>::iterator j;
    for (j = this->PDOObjects.begin(); j != this->PDOObjects.end(); j++)
    {
      if (j->second.length == this->objectSizes[i])
      {
        std::pair<IMCObjectName, IMCObject> nextObject{};
        nextObject.first = j->first;
        nextObject.second = j->second;
        this->sortedPDOObjects.push_back(nextObject);
        totalBits += this->objectSizes[i];
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
