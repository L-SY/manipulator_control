//
// Created by lsy on 25-6-4.
//

#pragma once

#include "can_actuator.h"
#include <stdexcept>

namespace device {

class CanRmActuator : public CanActuator {
public:
  CanRmActuator(const std::string& name,
                const std::string& bus,
                int id,
                const std::string& motor_type,
                const XmlRpc::XmlRpcValue& config);

  ~CanRmActuator() override = default;

  can_frame start() override;
  can_frame close() override;
  void read(const can_interface::CanFrameStamp& frameStamp) override;
  void readBuffer(const std::vector<can_interface::CanFrameStamp>& buffer) override;
  can_frame write() override;
  ActuatorCoefficients getCoefficientsFor() const override;

private:
  double max_out_;
  uint16_t q_raw_, q_raw_last_;
  int16_t qd_raw_;
};

} // namespace device
