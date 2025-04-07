//
// Created by lsy on 24-12-16.
//

#pragma once

#include "can_device.h"
#include "ros/ros.h"
#include <iostream>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <thread>

namespace device {

class ActuatorCoefficients {
public:
  double act2pos;
  double act2vel;
  double act2effort;
  double kp2act;
  double kd2act;
  double pos2act;
  double vel2act;
  double effort2act;
  double pos_offset;
  double vel_offset;
  double effort_offset;

  ActuatorCoefficients()
      : act2pos(0.), act2vel(0.), act2effort(0.),
        kp2act(0.), kd2act(0.), pos2act(0.),
        vel2act(0.), effort2act(0.),
        pos_offset(0.), vel_offset(0.), effort_offset(0.) {}

  ActuatorCoefficients(
      double act2pos, double act2vel, double act2effort,
      double kp2act, double kd2act, double pos2act,
      double vel2act, double effort2act,
      double pos_offset, double vel_offset, double effort_offset)
      : act2pos(act2pos), act2vel(act2vel), act2effort(act2effort),
        kp2act(kp2act), kd2act(kd2act), pos2act(pos2act),
        vel2act(vel2act), effort2act(effort2act),
        pos_offset(pos_offset), vel_offset(vel_offset), effort_offset(effort_offset) {}

  void printCoefficients() const {
    std::cout << "act2pos: " << act2pos << ", act2vel: " << act2vel
              << ", act2effort: " << act2effort << std::endl;
    std::cout << "kp2act: " << kp2act << ", kd2act: " << kd2act << std::endl;
    std::cout << "pos2act: " << pos2act << ", vel2act: " << vel2act
              << ", effort2act: " << effort2act << std::endl;
    std::cout << "pos_offset: " << pos_offset << ", vel_offset: " << vel_offset
              << ", effort_offset: " << effort_offset << std::endl;
  }
};

class ActuatorConfig {
private:
  std::unordered_map<std::string, ActuatorCoefficients> actuators;

public:
  ActuatorConfig() {
    actuators["dm4310"] = ActuatorCoefficients(
        0.00038147, 0.00510742, 0.0034188,
        8.19, 819.0, 2621.43812,
        195.793571, 292.500293,
        -12.5, -10.46, -7.0);

    actuators["dm4340"] = ActuatorCoefficients(
        0.00038147, 0.00265625, 0.01318359,
        8.19, 819.0, 2621.43812,
        376.470588, 75.8518734,
        -12.5, -5.44, -27.0);
  }

  const ActuatorCoefficients& getActuatorCoefficients(const std::string& actuator_name) const {
    if (actuators.find(actuator_name) != actuators.end()) {
      return actuators.at(actuator_name);
    } else {
      throw std::invalid_argument("Actuator not found: " + actuator_name);
    }
  }
};


enum class ControlMode {
  EFFORT,
  MIT,
  POSITION_VELOCITY
};

class CanDmActuator : public CanDevice {
public:
  CanDmActuator(const std::string& name, const std::string& bus, int id, const std::string& motor_type, const XmlRpc::XmlRpcValue& config);
  ~CanDmActuator() override = default;

  can_frame start() override;
  can_frame close() override;
  void read(const can_interface::CanFrameStamp& frameStamp) override;
  void readBuffer(const std::vector<can_interface::CanFrameStamp> &buffer) override;
  can_frame write() override;

  // Setters for commands
  void setCommand(double pos, double vel, double kp, double kd, double effort);
  void setControlMode(ControlMode mode) { control_mode_ = mode; }

  // Getters for state
  double getPosition() const { return position_; }
  double getVelocity() const { return velocity_; }
  double getEffort() const { return effort_; }
  double getFrequency() const { return frequency_; }
  ControlMode getControlMode() const { return control_mode_; }

  can_frame createRegisterFrame(uint8_t command, uint8_t reg_addr, uint32_t value = 0) const;

  uint32_t controlModeToMotorMode(ControlMode mode) const;
private:
  const ActuatorCoefficients coeff_;

  double position_{0};
  double velocity_{0};
  double effort_{0};
  double frequency_{0};
  ros::Time last_timestamp_;
  uint32_t seq_{0};

  double cmd_position_{0};
  double cmd_velocity_{0};
  double cmd_effort_{0};
  double cmd_kp_{0};
  double cmd_kd_{0};

  uint32_t master_id_;
  double max_velocity_{0}; // Maximum velocity in POSITION_VELOCITY mode

  ControlMode control_mode_{ControlMode::EFFORT};

  void updateFrequency(const ros::Time& stamp);
  can_frame writeEffortMIT();
  can_frame writePositionVelocity();
  void delayMicroseconds(unsigned int us);

  static constexpr uint8_t CTRL_MODE_REGISTER = 0x0A;

  static constexpr uint8_t CMD_READ = 0x33;
  static constexpr uint8_t CMD_WRITE = 0x55;
  static constexpr uint8_t CMD_SAVE = 0xAA;

  int start_call_count_;
};

} // namespace device
