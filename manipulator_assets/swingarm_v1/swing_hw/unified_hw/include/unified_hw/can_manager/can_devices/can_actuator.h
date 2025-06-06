// can_actuator.h

#pragma once

#include "can_device.h"
#include "unified_hw/common/lp_filter.h"
#include "ros/ros.h"
#include <XmlRpcValue.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <string>

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
      : act2pos(act2pos)
        , act2vel(act2vel)
        , act2effort(act2effort)
        , kp2act(kp2act)
        , kd2act(kd2act)
        , pos2act(pos2act)
        , vel2act(vel2act)
        , effort2act(effort2act)
        , pos_offset(pos_offset)
        , vel_offset(vel_offset)
        , effort_offset(effort_offset) {}

  inline double actuator2pos(uint16_t raw_act) const {
    return act2pos * static_cast<double>(raw_act) + pos_offset;
  }
  inline double actuator2vel(uint16_t raw_act) const {
    return act2vel * static_cast<double>(raw_act) + vel_offset;
  }
  inline double actuator2effort(uint16_t raw_act) const {
    return act2effort * static_cast<double>(raw_act) + effort_offset;
  }

  inline uint16_t pos2actuator(double phys_pos) const {
    double tmp = pos2act * (phys_pos - pos_offset);
    return static_cast<uint16_t>(tmp);
  }
  inline uint16_t vel2actuator(double phys_vel) const {
    double tmp = vel2act * (phys_vel - vel_offset);
    return static_cast<uint16_t>(tmp);
  }
  inline uint16_t effort2actuator(double phys_effort) const {
    double tmp = effort2act * (phys_effort - effort_offset);
    return static_cast<uint16_t>(tmp);
  }

  inline uint16_t kp2actuator(double phys_kp) const {
    return static_cast<uint16_t>(kp2act * phys_kp);
  }
  inline uint16_t kd2actuator(double phys_kd) const {
    return static_cast<uint16_t>(kd2act * phys_kd);
  }
};

namespace device {

enum class ControlMode {
  POSITION,
  VELOCITY,
  EFFORT,
  POSITION_VELOCITY,
  MIT
};

class CanActuator : public CanDevice {
public:
  CanActuator(const std::string& name,
              const std::string& bus,
              int id,
              const std::string& model,
              DeviceType type,
              const XmlRpc::XmlRpcValue& config = XmlRpc::XmlRpcValue())
      : CanDevice(name, bus, id, model, type, config),
        position_(0.0),
        velocity_(0.0),
        effort_(0.0),
        lp_filter_(100),
        control_mode_(ControlMode::EFFORT)
  {
    if (config.hasMember("master_id")) {
      master_id_ = static_cast<int>(config["master_id"]);
    } else {
      throw std::invalid_argument("Missing master_id in config");
    }

    if (config.hasMember("cutoff_freq")) {
        double new_cutoff = static_cast<double>(config["cutoff_freq"]);
        cutoff_freq_ = new_cutoff;
        lp_filter_ = LowPassFilter{cutoff_freq_};
    } else {
      ROS_INFO("No cutoff_freq in config; using default = 100");
    }

    if (config.hasMember("control_mode")) {
      std::string mode_str = static_cast<std::string>(config["control_mode"]);
      if (mode_str == "MIT") {
        control_mode_ = ControlMode::MIT;
      } else if (mode_str == "POSITION_VELOCITY") {
        control_mode_ = ControlMode::POSITION_VELOCITY;
      } else if (mode_str == "VELOCITY") {
        control_mode_ = ControlMode::VELOCITY;
      } else if (mode_str == "POSITION") {
        control_mode_ = ControlMode::POSITION;
      } else {
        control_mode_ = ControlMode::EFFORT;
      }
    }

  }

  virtual ~CanActuator() override = default;

  virtual ActuatorCoefficients getCoefficientsFor() const = 0;
  virtual can_frame start() override = 0;
  virtual can_frame close() override = 0;
  virtual void read(const can_interface::CanFrameStamp& frameStamp) override = 0;
  virtual void readBuffer(const std::vector<can_interface::CanFrameStamp>& buffer) override = 0;
  virtual can_frame write() override = 0;

  void setCommand(double pos, double vel, double kp, double kd, double effort) {
    cmd_position_ = pos;
    cmd_velocity_ = vel;
    cmd_kp_       = kp;
    cmd_kd_       = kd;
    cmd_effort_   = effort;
  }

  void setControlMode(ControlMode mode) {
    control_mode_ = mode;
  }

  double* getPositionPtr()   { return &position_; }
  double* getVelocityPtr()   { return &velocity_; }
  double* getEffortPtr()     { return &effort_; }
  double* getCmdPosPtr()     { return &cmd_position_; }
  double* getCmdVelPtr()     { return &cmd_velocity_; }
  double* getCmdEffortPtr()  { return &cmd_effort_; }
  double* getCmdKpPtr()      { return &cmd_kp_; }
  double* getCmdKdPtr()      { return &cmd_kd_; }

  double getPosition() const   { return position_; }
  double getVelocity() const   { return velocity_; }
  double getEffort() const     { return effort_; }
  double getMasterID() const     { return master_id_; }
  ControlMode getControlMode() const { return control_mode_; }

protected:
  ActuatorCoefficients coeff_;
  double position_, position_last_;
  double velocity_;
  double effort_;
  double frequency_;
  uint32_t seq_;
  ros::Time last_timestamp_;

  double cmd_position_;
  double cmd_velocity_;
  double cmd_effort_;
  double cmd_kp_;
  double cmd_kd_;

  uint32_t master_id_;
  ControlMode control_mode_;
  double cutoff_freq_;
  LowPassFilter lp_filter_;

  uint8_t temp_;
  int64_t q_circle_;
};

} // namespace device
