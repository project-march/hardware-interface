// Copyright 2019 Project March.
#include <march_hardware/PDOmap.h>

#include <map>
#include <utility>

namespace march4cpp
{
std::unordered_map<IMCObjectName, IMCObject> PDOmap::all_objects = {
  { IMCObjectName::StatusWord, IMCObject(0x6041, 16) },
  { IMCObjectName::ActualPosition, IMCObject(0x6064, 32) },
  { IMCObjectName::MotionErrorRegister, IMCObject(0x2000, 16) },
  { IMCObjectName::DetailedErrorRegister, IMCObject(0x2002, 16) },
  { IMCObjectName::DCLinkVoltage, IMCObject(0x2055, 16) },
  { IMCObjectName::DriveTemperature, IMCObject(0x2058, 16) },
  { IMCObjectName::ActualTorque, IMCObject(0x6077, 16) },
  { IMCObjectName::CurrentLimit, IMCObject(0x207F, 16) },
  { IMCObjectName::MotorPosition, IMCObject(0x2088, 32) },
  { IMCObjectName::ControlWord, IMCObject(0x6040, 16) },
  { IMCObjectName::TargetPosition, IMCObject(0x607A, 32) },
  { IMCObjectName::TargetTorque, IMCObject(0x6071, 16) },
  { IMCObjectName::QuickStopDeceleration, IMCObject(0x6085, 32) },
  { IMCObjectName::QuickStopOption, IMCObject(0x605A, 16) }
};

void PDOmap::add_object(IMCObjectName object_name)
{
  if (PDOmap::all_objects.find(object_name) == PDOmap::all_objects.end())
  {
    ROS_WARN("IMC does not exists in in initialised objects so can not be added to PDO");
    return;
  }

  if (this->PDO_objects.count(object_name) != 0)
  {
    ROS_WARN("IMC object %i is already added to PDO map", object_name);
    return;
  }

  this->PDO_objects.insert({ object_name, PDOmap::all_objects[object_name] });  // NOLINT(whitespace/braces)

  int total_used_bits = 0;
  for (std::pair<IMCObjectName, IMCObject> PDO_object : this->PDO_objects)
  {
    total_used_bits = total_used_bits + PDO_object.second.length;
  }

  if (total_used_bits > this->nr_of_regs * this->bits_per_register)
  {
    ROS_FATAL("Too many objects in PDO Map (total bits %d, only %d allowed), PDO object: %i could not be added",
              total_used_bits, this->nr_of_regs * this->bits_per_register, object_name);
    throw std::exception();
  }
}

std::map<IMCObjectName, int> PDOmap::map(int slaveIndex, enum dataDirection direction)
{
  this->sort_PDO_objects();

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
  std::map<IMCObjectName, int> byte_offsets;

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
    byte_offsets[nextObject.first] = byteOffset;

    sdo_bit32(slaveIndex, currentReg, counter,
              PDOmap::combineAddressLength(nextObject.second.address, nextObject.second.length));
    counter++;
  }

  // Make sure the last PDO is activated
  sdo_bit8(slaveIndex, currentReg, 0, counter - 1);

  // Deactivated the sync manager and configure with the new PDO
  sdo_bit8(slaveIndex, baseSyncManager, 0, 0);
  int currentPDONr = (currentReg - baseRegister) + 1;
  sdo_bit16(slaveIndex, baseSyncManager, currentPDONr, currentReg);

  // Explicitly disable PDO registers which are not used
  currentReg++;
  if (currentReg <= (baseRegister + nr_of_regs))
  {
    for (int unusedRegister = currentReg; unusedRegister <= (baseRegister + this->nr_of_regs); unusedRegister++)
    {
      sdo_bit8(slaveIndex, unusedRegister, 0, 0);
    }
  }

  // Activate the sync manager again
  int totalAmountPDO = (currentReg - baseRegister);
  sdo_bit8(slaveIndex, baseSyncManager, 0, totalAmountPDO);

  return byte_offsets;
}

void PDOmap::sort_PDO_objects()
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
}

uint32_t PDOmap::combineAddressLength(uint16_t address, uint16_t length)
{
  uint32_t MSword = ((address & 0xFFFF) << 16);  // Shift 16 bits left for most significant word
  uint32_t LSword = (length & 0xFFFF);
  return (MSword | LSword);
}

}  // namespace march4cpp
