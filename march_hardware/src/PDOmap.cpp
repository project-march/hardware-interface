// Copyright 2019 Project March.
#include <march_hardware/PDOmap.h>

#include <map>
#include <utility>

namespace march4cpp
{
PDOmap::PDOmap()
{
  this->all_objects[IMCObjectName::StatusWord] = IMCObject(0x6041, 16);
  this->all_objects[IMCObjectName::ActualPosition] = IMCObject(0x6064, 32);
  this->all_objects[IMCObjectName::MotionErrorRegister] = IMCObject(0x2000, 16);
  this->all_objects[IMCObjectName::DetailedErrorRegister] = IMCObject(0x2002, 16);
  this->all_objects[IMCObjectName::DCLinkVoltage] = IMCObject(0x2055, 16);
  this->all_objects[IMCObjectName::DriveTemperature] = IMCObject(0x2058, 16);
  this->all_objects[IMCObjectName::ActualTorque] = IMCObject(0x6077, 16);
  this->all_objects[IMCObjectName::CurrentLimit] = IMCObject(0x207F, 16);
  this->all_objects[IMCObjectName::MotorPosition] = IMCObject(0x2088, 32);
  this->all_objects[IMCObjectName::ControlWord] = IMCObject(0x6040, 16);
  this->all_objects[IMCObjectName::TargetPosition] = IMCObject(0x607A, 32);
  this->all_objects[IMCObjectName::TargetTorque] = IMCObject(0x6071, 16);
  this->all_objects[IMCObjectName::QuickStopDeceleration] = IMCObject(0x6085, 32);
  this->all_objects[IMCObjectName::QuickStopOption] = IMCObject(0x605A, 16);
}

void PDOmap::addObject(IMCObjectName object_name)
{
  if (this->all_objects.count(object_name) != 1)
  {
    ROS_WARN("IMC object does not exist (yet), or multiple exist");
    return;
  }
  else if (this->PDO_objects.count(object_name) != 0)
  {
    ROS_WARN("IMC object is already added to PDO map");
    return;
  }
  else
  {
    this->PDO_objects[object_name] = this->all_objects[object_name];
  }
}

std::map<enum IMCObjectName, int> PDOmap::map(int slaveIndex, enum dataDirection direction)
{
  this->sortPDOObjects();

  if (direction == dataDirection::miso)
  {
    return configurePDO(slaveIndex, 0x1A00, 0x1C13);
  }
  else if (direction == dataDirection::mosi)
  {
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
  int sizeLeft = this->bits_per_register;

  sdo_bit8(slaveIndex, baseRegister, 0, 0);

  for (const auto& nextObject : sorted_PDO_objects)
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
      if (currentReg > (baseRegister + nr_of_regs))
      {
        ROS_ERROR("Amount of registers was overwritten, amount of parameters does not fit in the PDO messages.");
      }

      sizeLeft = this->bits_per_register - nextObject.second.length;
      counter = 1;

      sdo_bit8(slaveIndex, currentReg, 0, 0);
    }

    int byteOffset = (bits_per_register - (sizeLeft + nextObject.second.length)) / 8;
    this->byte_offsets[nextObject.first] = byteOffset;

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
  if (currentReg <= (baseRegister + nr_of_regs))
  {
    for (int unusedRegister = currentReg; unusedRegister < baseRegister + this->nr_of_regs; unusedRegister++)
    {
      sdo_bit8(slaveIndex, unusedRegister, 0, 0);
    }
  }

  // Active the sync manager again
  int totalAmountPDO = (currentReg - baseRegister);
  sdo_bit8(slaveIndex, baseSyncManager, 0, totalAmountPDO);

  return this->byte_offsets;
}

void PDOmap::sortPDOObjects()
{
  int totalBits = 0;
  for (int objectSize : this->object_sizes)
  {
    for (const auto& object : PDO_objects)
    {
      if (object.second.length == objectSize)
      {
        this->sorted_PDO_objects.emplace_back(object);
        totalBits += objectSize;
      }
    }
  }

  if (totalBits > this->nr_of_regs * this->bits_per_register)
  {
    ROS_FATAL("Too many objects in PDO Map (total bits %d, only %d allowed)", totalBits,
              this->nr_of_regs * this->bits_per_register);
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
