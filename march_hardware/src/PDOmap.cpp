// Copyright 2019 Project March.
#include <march_hardware/PDOmap.h>

namespace march4cpp {
PDOmap::PDOmap() { this->initIMCObjects(); }

void PDOmap::addObject(IMCObjectName objectname, int reg) {
  if (this->allObjects.count(objectname) != 1) {
    ROS_WARN("IMC object does not exist (yet), or multiple exist");
    return;
  } else if (find(this->mappedIMCObjects.begin(), this->mappedIMCObjects.end(),
                  objectname) != this->mappedIMCObjects.end()) {
    ROS_WARN("IMC object is already added to PDO map");
    return;
  } else if (reg < 1 || reg > 4) {
    ROS_WARN("Register %d invalid. 4 registers available", reg);
    return;
  } else { // Valid
    this->mappedIMCObjects.push(objectname);
    this->mappedIMCObjectRegisters.push(reg);
  }
}

std::map<enum IMCObjectName, int> PDOmap::map(int slaveIndex,
                                              enum dataDirection direction) {
  int currentRegister;
  int SMAddress;
  if (direction == dataDirection::miso) {
    currentRegister = 0x1A00;
    SMAddress = 0x1C13;
  } else if (direction == dataDirection::mosi) {
    currentRegister = 0x1600;
    SMAddress = 0x1C12;
  } else {
    ROS_ERROR("Invalid dataDirection argument");
  }
  // Clear SyncManager Object
  sdo_bit8(slaveIndex, SMAddress, 0, 0);
  int currentRegObjectCount = 0;
  int bitsLeftCurrentRegister = this->bitsPerReg;
  int byteOffset = 0;
  while (!this->mappedIMCObjects.empty()) {
    if
      IMCObjectName objectToBeMapped = this->mappedIMCObjects.pop();

    currentRegObjectCount = this->mapObject(
        objectToBeMapped, currentRegObjectCount, currentRegister, slaveIndex);
  }

  //  int startReg = reg;
  //  int lastFilledReg = reg;
  //  int sizeleft = this->bitsPerReg;
  //  int counter = 0;
  //  int byteOffset = 0;
  //  while (this->sortedPDOObjects.size() > 0)
  //  {
  //    // Check if register is still empty
  //    if (sizeleft == this->bitsPerReg)
  //    {
  //      sdo_bit32(slaveIndex, reg, 0, 0);
  //    }
  //    // Get next object (from end, because sorted from small to large)
  //    std::pair<IMCObjectName, IMCObject> nextObject =
  //    this->sortedPDOObjects.back();
  //    this->sortedPDOObjects.pop_back();
  //    // Add next object to map
  //    counter++;
  //    sdo_bit32(slaveIndex, reg, counter,
  //              this->combineAddressLength(nextObject.second.address,
  //              nextObject.second.length));
  //    this->byteOffsets[nextObject.first] = byteOffset;
  //    byteOffset += nextObject.second.length / 8;
  //    sizeleft -= nextObject.second.length;
  //    // Check if this was the last object of the list
  //    if (this->sortedPDOObjects.size() == 0)
  //    {
  //      sdo_bit32(slaveIndex, reg, 0, counter);
  //      lastFilledReg = reg;
  //      reg++;
  //    }
  //    // else, check if register is full
  //    else if (sizeleft <= 0)
  //    {
  //      sdo_bit32(slaveIndex, reg, 0, counter);
  //      reg++;
  //      counter = 0;
  //      sizeleft = this->bitsPerReg;
  //    }
  //  }
  //  // For the unused registers, set count to zero
  //  for (int i = reg; i < startReg + this->nrofRegs; i++)
  //  {
  //    sdo_bit32(slaveIndex, i, 0, 0);
  //  }
  //  // For all filled registers, set data to Sync Manager object
  //  int count = 0;
  //  for (int i = startReg; i <= lastFilledReg; i++)
  //  {
  //    count++;
  //    sdo_bit16(slaveIndex, SMAddress, count, 0x1600);
  //  }
  //  sdo_bit8(slaveIndex, SMAddress, 0, count);
  //  return this->byteOffsets;
}

int mapObject(IMCObjectName objectName, int objectCount, int reg,
              int slaveIndex) {
  objectCount++;
  IMCObject object = this->imcObjects[objectName];
  sdo_bit32(slaveIndex, reg, objectCount,
            this->combineAddressLength(object.address, object.length));
  sdo_bit32(slaveIndex, reg, 0, currentRegister);
  return objectCount;
}

uint32_t PDOmap::combineAddressLength(uint16_t address, uint16_t length) {
  uint32_t MSword = ((address & 0xFFFF)
                     << 16); // Shift 16 bits left for most significant word
  uint32_t LSword = (length & 0xFFFF);
  return (MSword | LSword);
}

void PDOmap::initIMCObjects() {
  // Object(address, length, name);
  this->imcObjects[IMCObjectName::StatusWord] = IMCObject(0x6041, 16);
  this->imcObjects[IMCObjectName::ActualPosition] = IMCObject(0x6064, 32);
  this->imcObjects[IMCObjectName::MotionErrorRegister] = IMCObject(0x2000, 16);
  this->imcObjects[IMCObjectName::DetailedErrorRegister] =
      IMCObject(0x2002, 16);
  this->imcObjects[IMCObjectName::DCLinkVoltage] = IMCObject(0x2055, 16);
  this->imcObjects[IMCObjectName::DriveTemperature] = IMCObject(0x2058, 16);
  this->imcObjects[IMCObjectName::ActualTorque] = IMCObject(0x6077, 16);
  this->imcObjects[IMCObjectName::CurrentLimit] = IMCObject(0x207F, 16);
  this->imcObjects[IMCObjectName::MotorPosition] = IMCObject(0x2088, 32);
  this->imcObjects[IMCObjectName::ControlWord] = IMCObject(0x6040, 16);
  this->imcObjects[IMCObjectName::TargetPosition] = IMCObject(0x607A, 32);
  this->imcObjects[IMCObjectName::TargetCurrent] = IMCObject(0x6071, 16);
  this->imcObjects[IMCObjectName::QuickStopDeceleration] =
      IMCObject(0x6085, 32);
  this->imcObjects[IMCObjectName::QuickStopOption] = IMCObject(0x605A, 16);
  // etc...
  // If a new entry is added here, first add it to the enum (in the header
  // file)!
}
} // namespace march4cpp
