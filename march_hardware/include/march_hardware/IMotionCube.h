// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_IMOTIONCUBE_H
#define MARCH_HARDWARE_IMOTIONCUBE_H
#include "march_hardware/ActuationMode.h"
#include "march_hardware/EncoderAbsolute.h"
#include "march_hardware/EncoderIncremental.h"
#include "march_hardware/EtherCAT/EthercatIO.h"
#include "march_hardware/IMotionCubeState.h"
#include "march_hardware/IMotionCubeTargetState.h"
#include "march_hardware/PDOmap.h"
#include "march_hardware/Slave.h"

#include <unordered_map>
#include <string>

namespace march
{
class IMotionCube : public Slave
{
public:
  IMotionCube(int slave_index, EncoderAbsolute encoder_absolute, EncoderIncremental encoder_incremental,
              ActuationMode actuation_mode);

  ~IMotionCube() = default;

  void writeInitialSDOs(int cycle_time) override;

  double getAngleRadAbsolute();
  double getAngleRadIncremental();
  int16_t getTorque();
  int32_t getAngleIUAbsolute();
  int32_t getAngleIUIncremental();

  uint16_t getStatusWord();
  uint16_t getMotionError();
  uint16_t getDetailedError();

  ActuationMode getActuationMode() const;

  float getMotorCurrent();
  float getMotorVoltage();

  void setControlWord(uint16_t control_word);

  void actuateRad(double target_rad);
  void actuateTorque(int16_t target_torque);

  void goToTargetState(IMotionCubeTargetState target_state);
  void goToOperationEnabled();

  /** @brief Override comparison operator */
  friend bool operator==(const IMotionCube& lhs, const IMotionCube& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.encoder_absolute_ == rhs.encoder_absolute_ &&
           lhs.encoder_incremental_ == rhs.encoder_incremental_;
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const IMotionCube& imc)
  {
    return os << "slaveIndex: " << imc.slaveIndex << ", "
              << "encoderIncremental: " << imc.encoder_incremental_ << ", "
              << "encoderAbsolute: " << imc.encoder_absolute_;
  }

  constexpr static double MAX_TARGET_DIFFERENCE = 0.393;
  // This value is slightly larger than the current limit of the
  // linear joints defined in the URDF.
  const static int16_t MAX_TARGET_TORQUE = 23500;

  // Watchdog base time = 1 / 25 MHz * (2498 + 2) = 0.0001 seconds=100 µs
  static const uint16_t WATCHDOG_DIVIDER = 2498;
  // 500 * 100us = 50 ms = watchdog timer
  static const uint16_t WATCHDOG_TIME = 500;

private:
  void actuateIU(int32_t target_iu);

  void mapMisoPDOs();
  void mapMosiPDOs();
  void writeInitialSettings(uint8_t cycle_time);

  EncoderAbsolute encoder_absolute_;
  EncoderIncremental encoder_incremental_;
  ActuationMode actuation_mode_;

  std::unordered_map<IMCObjectName, uint8_t> miso_byte_offsets_;
  std::unordered_map<IMCObjectName, uint8_t> mosi_byte_offsets_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_IMOTIONCUBE_H
