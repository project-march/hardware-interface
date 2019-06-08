// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include <march_hardware/PDOmap.h>
#include <march_hardware/MarchRobot.h>

class PDOTest : public ::testing::Test
{
protected:
};

TEST_F(PDOTest, sortPDOmap)
{
  march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();

  pdoMapMISO.addObject(march4cpp::IMCObjectName::ActualPosition, 1);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::StatusWord, 3);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::ControlWord, 1);
  std::map<march4cpp::IMCObjectName, int> misoByteOffsets = pdoMapMISO.map(1, march4cpp::dataDirection::miso);


  ASSERT_EQ(0, misoByteOffsets[march4cpp::IMCObjectName::ActualPosition]);
  ASSERT_EQ(16, misoByteOffsets[march4cpp::IMCObjectName::StatusWord]);
  ASSERT_EQ(2, misoByteOffsets[march4cpp::IMCObjectName::ControlWord]);
}

TEST_F(PDOTest, invalidRegisters)
{
  march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();

  pdoMapMISO.addObject(march4cpp::IMCObjectName::ActualPosition,5);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::StatusWord, -1);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::ControlWord, 0);
  std::map<march4cpp::IMCObjectName, int> misoByteOffsets = pdoMapMISO.map(2, march4cpp::dataDirection::miso);
  ASSERT_EQ(0, misoByteOffsets.size());
}

TEST_F(PDOTest, ObjectCounts)
{
    march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();
    pdoMapMISO.addObject(march4cpp::IMCObjectName::CurrentLimit, 1);
    std::map<march4cpp::IMCObjectName, int> misoByteOffsets = pdoMapMISO.map(3, march4cpp::dataDirection::miso);
    ASSERT_EQ(1, misoByteOffsets.count(march4cpp::IMCObjectName::CurrentLimit));
    ASSERT_EQ(0, misoByteOffsets.count(march4cpp::IMCObjectName::DCLinkVoltage));
}

TEST_F(PDOTest, exceedMaxSize)
{
  march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();
  pdoMapMISO.addObject(march4cpp::IMCObjectName::StatusWord, 1);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::ActualPosition, 1);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::ControlWord, 1);
  pdoMapMISO.addObject(march4cpp::IMCObjectName::DetailedErrorRegister, 1);
  ASSERT_THROW(pdoMapMISO.map(4, march4cpp::dataDirection::miso), std::exception);
}
