// can_dm_actuator.h

#pragma once

#include "can_actuator.h"
#include <stdexcept>

namespace device {

class CanDmActuator : public CanActuator {
public:
  CanDmActuator(const std::string& name,
                const std::string& bus,
                int id,
                const std::string& motor_type,
                const XmlRpc::XmlRpcValue& config);

  ~CanDmActuator() override = default;

  can_frame start() override;
  can_frame close() override;
  void read(const can_interface::CanFrameStamp& frameStamp) override;
  void readBuffer(const std::vector<can_interface::CanFrameStamp>& buffer) override;
  can_frame write() override;

  can_frame createRegisterFrame(uint8_t command, uint8_t reg_addr, uint32_t value = 0) const;

  uint32_t controlModeToMotorMode(ControlMode mode) const;

  can_frame writeEffortMIT();
  can_frame writePositionVelocity();
  ActuatorCoefficients getCoefficientsFor() const override;

private:
  double max_velocity_{0};
  int start_call_count_{0};

  static constexpr uint8_t CTRL_MODE_REGISTER = 0x0A;
  static constexpr uint8_t CMD_READ  = 0x33;
  static constexpr uint8_t CMD_WRITE = 0x55;
  static constexpr uint8_t CMD_SAVE  = 0xAA;
};

} // namespace device
