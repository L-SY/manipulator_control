//
// Created by lsy on 25-6-4.
//

#pragma once

#include "can_device.h"
#include "ros/ros.h"
#include <iostream>
#include <unordered_map>
#include <stdexcept>
#include <cstdint>
#include "actuator.h"

namespace device {

struct RmActuatorConfigEntry {
  ActuatorCoefficients coeff;
  uint16_t            max_out;
};

class RmActuatorConfig {
private:
  std::unordered_map<std::string, RmActuatorConfigEntry> actuators_;

public:
  RmActuatorConfig() {
    actuators_["rm_3508"] = RmActuatorConfigEntry{
        ActuatorCoefficients(
            /* act2pos      */ 0.0007669903,  // 2π/8192
            /* act2vel      */ 0.1047197551,  // 2π/60
            /* act2effort   */ 1.90702994e-5, // 20/16384 * 0.0156223893
            /* kp2act       */ 0.0,
            /* kd2act       */ 0.0,
            /* pos2act      */ 0.0,
            /* vel2act      */ 0.0,
            /* effort2act   */ 52437.561519,  // 1 / 1.90702994e-5
            /* pos_offset   */ 0.0,
            /* vel_offset   */ 0.0,
            /* effort_offset*/ 0.0
            ),
        /* max_out = */ 16384
    };

    actuators_["rm_6020"] = RmActuatorConfigEntry{
        ActuatorCoefficients(
            /* act2pos      */ 0.0007670840,
            /* act2vel      */ 0.1047197551,
            /* act2effort   */ 5.880969e-5,
            /* kp2act       */ 0.0,
            /* kd2act       */ 0.0,
            /* pos2act      */ 0.0,
            /* vel2act      */ 0.0,
            /* effort2act   */ 25000.0,
            /* pos_offset   */ 0.0,
            /* vel_offset   */ 0.0,
            /* effort_offset*/ 0.0
            ),
        /* max_out = */ 30000
    };

    actuators_["rm_2006"] = RmActuatorConfigEntry{
        ActuatorCoefficients(
            /* act2pos      */ 2.13078897e-5, // 2π/8192*(1/36)
            /* act2vel      */ 0.0029088820,  // 2π/60*(1/36)
            /* act2effort   */ 0.00018,       // 10/10000*0.18
            /* kp2act       */ 0.0,
            /* kd2act       */ 0.0,
            /* pos2act      */ 0.0,
            /* vel2act      */ 0.0,
            /* effort2act   */ 5555.5555555,  // 1 / 0.00018
            /* pos_offset   */ 0.0,
            /* vel_offset   */ 0.0,
            /* effort_offset*/ 0.0
            ),
        /* max_out = */ 10000
    };
  }

  const RmActuatorConfigEntry& getRmActuatorConfig(const std::string& name) const {
    auto it = actuators_.find(name);
    if (it == actuators_.end()) {
      throw std::invalid_argument("RmActuatorConfig: actuator not found → " + name);
    }
    return it->second;
  }
};

class CanRmActuator : public CanDevice {
public:
  CanRmActuator(const std::string& name,
                const std::string& bus,
                int id,
                const std::string& motor_type,
                const XmlRpc::XmlRpcValue& config);

  ~CanRmActuator() override = default;

  can_frame start() override;
  can_frame close() override;
  void     read(const can_interface::CanFrameStamp& frameStamp) override;
  void     readBuffer(const std::vector<can_interface::CanFrameStamp>& buffer) override;
  can_frame write() override;

//  void setCommand(double pos, double vel, double effort);

  double       getPosition()   const { return position_; }
  double       getVelocity()   const { return velocity_; }
  double       getEffort()     const { return effort_; }
  double       getFrequency()  const { return frequency_; }

private:
  std::string name_;
  std::string motor_type_;

  RmActuatorConfigEntry coeff_;
  uint16_t             max_out_;

  double       position_{0.0};
  double       velocity_{0.0};
  double       effort_{0.0};
  double       frequency_{0.0};
  ros::Time    last_timestamp_;
  uint32_t     seq_{0};

  uint16_t     cmd_pos_act_{0};
  uint16_t     cmd_vel_act_{0};
  uint16_t     cmd_effort_act_{0};
};

} // namespace device

