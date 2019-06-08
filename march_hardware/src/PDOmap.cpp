// Copyright 2019 Project March.
#include <march_hardware/PDOmap.h>

namespace march4cpp
{
PDOmap::PDOmap() { this->initIMCObjects(); }

void PDOmap::addObject(IMCObjectName objectname, int reg)
{
  if (reg < 1 || reg > 4)
  {
    ROS_WARN(
        "Indicated register %d is invalid. There are 4 PDO registers available",
        reg);
  }
  else
  { // Valid
    this->mappedIMCObjects.push(objectname);
    this->mappedIMCObjectRegisters.push(reg);
  }
}

std::map<enum IMCObjectName, int> PDOmap::map(int slaveIndex, enum dataDirection direction)
{
  int firstRegister;
  int SMAddress;
  if (direction == dataDirection::miso)
  {
    firstRegister = 0x1A00;
    SMAddress = 0x1C13;
  }
  else if (direction == dataDirection::mosi)
  {
    firstRegister = 0x1600;
    SMAddress = 0x1C12;
  }
  else
  {
    ROS_ERROR("Invalid dataDirection argument");
  }
  // Clear SyncManager Object
  sdo_bit8(slaveIndex, SMAddress, 0, 0);
  // Initial object counts and bits left per register
  int registerObjectCount[4] = {0, 0, 0, 0};
  int registerBitsLeft[4] = {64, 64, 64, 64};
  // Write initial object counts of zero to all registers
  for (int i = firstRegister; i < firstRegister + 4; i++)
  {
    sdo_bit32(slaveIndex, i, 0, 0);
  }
  // Get to-be-mapped objects in a FIFO manner and map them
  while (!this->mappedIMCObjects.empty())
  {
    // Get next object
    IMCObject objectToBeMapped =
        this->imcObjects[this->mappedIMCObjects.front()];
    IMCObjectName objectToBeMappedName = this->mappedIMCObjects.front();
    this->mappedIMCObjects.pop();
    int desiredRegisterNr = this->mappedIMCObjectRegisters.front();
    this->mappedIMCObjectRegisters.pop();
    int desiredRegister = desiredRegisterNr + firstRegister - 1;
    // Check if the object fits in the desired register
    if (objectToBeMapped.length > registerBitsLeft[desiredRegisterNr - 1])
    {
      ROS_FATAL("IMC object 0x%X cannot be mapped because it does not fit in "
                "register 0x%X any more. Try another register",
                objectToBeMapped.address, desiredRegister);
      throw std::exception();
    }
    // Map the object
    registerObjectCount[desiredRegisterNr - 1]++;
    this->mapObject(objectToBeMapped,
                    registerObjectCount[desiredRegisterNr - 1], desiredRegister,
                    slaveIndex);
    this->byteOffsets[objectToBeMappedName] =
        8 * desiredRegisterNr - (registerBitsLeft[desiredRegisterNr - 1] / 8);
    registerBitsLeft[desiredRegisterNr - 1] -= objectToBeMapped.length;
  }
  // Tell the IMC how many objects are mapped in each register
  for (int i = firstRegister; i < firstRegister + 4; i++)
  {
    sdo_bit32(slaveIndex, i, 0, registerObjectCount[i - firstRegister]);
  }
  // Update the SyncManager
  int nrOfRegsWithObjects = 0;
  for (int i = 0; i < 4; i++)
  {
    if (registerObjectCount[i] > 0)
    {
      // There is at least one object in this register
      nrOfRegsWithObjects++;
      sdo_bit16(slaveIndex, SMAddress, nrOfRegsWithObjects, firstRegister + i);
    }
  }
  sdo_bit8(slaveIndex, SMAddress, 0, nrOfRegsWithObjects);
  // Return the byteoffsets
  return this->byteOffsets;
}

void PDOmap::mapObject(IMCObject object, int objectCount, int reg, int slaveIndex)
  {
  uint32_t combinedAddressLength =
      this->combineAddressLength(object.address, object.length);
  // Map the object
  bool success = sdo_bit32(slaveIndex, reg, objectCount, combinedAddressLength);
  // Update count for the register
  // TODO(Martijn, BaCo) find out if next line is necessary in some cases
  //  sdo_bit32(slaveIndex, reg, 0, currentRegister);
  // Check if sdo write was successful, if not throw error
  if (!success)
  {
    ROS_FATAL("SDO write when PDO mapping object 0x%X to IMC %d failed",
              object.address, slaveIndex);
    throw std::exception();
  }
}

uint32_t PDOmap::combineAddressLength(uint16_t address, uint16_t length)
{
  uint32_t MSword = ((address & 0xFFFF) << 16);  // Shift 16 bits left for most significant word
  uint32_t LSword = (length & 0xFFFF);
  return (MSword | LSword);
}

void PDOmap::initIMCObjects()
{
  // Object(address, length);
  this->imcObjects[IMCObjectName::StatusWord] = IMCObject(0x6041, 16);
  this->imcObjects[IMCObjectName::ActualPosition] = IMCObject(0x6064, 32);
  this->imcObjects[IMCObjectName::MotionErrorRegister] = IMCObject(0x2000, 16);
  this->imcObjects[IMCObjectName::DetailedErrorRegister] = IMCObject(0x2002, 16);
  this->imcObjects[IMCObjectName::DCLinkVoltage] = IMCObject(0x2055, 16);
  this->imcObjects[IMCObjectName::DriveTemperature] = IMCObject(0x2058, 16);
  this->imcObjects[IMCObjectName::ActualTorque] = IMCObject(0x6077, 16);
  this->imcObjects[IMCObjectName::CurrentLimit] = IMCObject(0x207F, 16);
  this->imcObjects[IMCObjectName::MotorPosition] = IMCObject(0x2088, 32);
  this->imcObjects[IMCObjectName::ControlWord] = IMCObject(0x6040, 16);
  this->imcObjects[IMCObjectName::TargetPosition] = IMCObject(0x607A, 32);
  this->imcObjects[IMCObjectName::TargetCurrent] = IMCObject(0x6071, 16);
  this->imcObjects[IMCObjectName::QuickStopDeceleration] = IMCObject(0x6085, 32);
  this->imcObjects[IMCObjectName::QuickStopOption] = IMCObject(0x605A, 16);
  // etc...
  // If a new entry is added here, first add it to the enum in the header file!
}
}  // namespace march4cpp
