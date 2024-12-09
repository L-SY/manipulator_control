//
// Created by lsy on 24-8-28.
//

// ref:
// https://github.com/ros-controls/ros_controllers/blob/noetic-devel/gripper_action_controller/include/gripper_action_controller/hardware_interface_adapter
#pragma once

#include <cassert>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <robot_common/interface/hardware_interface/HybridJointInterface.h>
#include <manipulator_msgs/GripperCmd.h>

namespace gripper_controller {
typedef realtime_tools::RealtimeBuffer<manipulator_msgs::GripperCmd> CmdBuff;

template <class HardwareInterface> class HardwareInterfaceAdapter {
public:
  bool init(hardware_interface::JointHandle &joint_handle,
            ros::NodeHandle &controller_nh, std::shared_ptr<CmdBuff> cmdRtBuffer) {
    return false;
  }

  void starting(const ros::Time &time) {}
  void stopping(const ros::Time &time) {}

  void updateCommand(const ros::Time &time, const ros::Duration &period) {}
};

template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface> {
public:
  HardwareInterfaceAdapter() : joint_handle_ptr_(nullptr) {}

  bool init(hardware_interface::JointHandle &joint_handle,
            ros::NodeHandle &controller_nh, std::shared_ptr<CmdBuff> cmdRtBuffer) {
    CmdBuff_ = std::move(cmdRtBuffer);
    // Store pointer to joint handles
    joint_handle_ptr_ = &joint_handle;

    // Initialize PIDs
    ros::NodeHandle joint_nh(ros::NodeHandle(controller_nh, "pid"));

    // Init PID gains from ROS parameter server
    pid_.reset(new control_toolbox::Pid());
    if (!pid_->init(joint_nh)) {
      ROS_WARN_STREAM(
          "Failed to initialize PID gains from ROS parameter server.");
      return false;
    }

    return true;
  }

  void starting(const ros::Time &time) {
    if (!joint_handle_ptr_) {
      return;
    }
    // Reset PIDs, zero effort commands
    pid_->reset();
    (*joint_handle_ptr_).setCommand(0.0);
  }

  void stopping(const ros::Time &time) {}

  void updateCommand(const ros::Time &time, const ros::Duration &period) {
    // Update PIDs
    double des_position = (CmdBuff_->readFromRT())->des_pos;
    double des_velocity = (CmdBuff_->readFromRT())->des_vel;
    double posErr, velErr;

    double current_position = joint_handle_ptr_->getPosition();
    double current_velocity = joint_handle_ptr_->getVelocity();

    //    enforceJointLimits(des_position);

    posErr = des_position - current_position;
    velErr = des_velocity - current_velocity;
    double command = pid_->computeCommand(posErr, velErr, period);
    (*joint_handle_ptr_).setCommand(command);
  }

private:
  typedef std::shared_ptr<control_toolbox::Pid> PidPtr;
  PidPtr pid_;
  hardware_interface::JointHandle *joint_handle_ptr_;
  std::shared_ptr<CmdBuff> CmdBuff_;
};

template <>
class HardwareInterfaceAdapter<hardware_interface::HybridJointInterface> {
public:
  HardwareInterfaceAdapter() : joint_handle_ptr_(nullptr) {}

  bool init(hardware_interface::HybridJointHandle &joint_handle,
            ros::NodeHandle &controller_nh, std::shared_ptr<CmdBuff> cmdRtBuffer) {
    CmdBuff_ = std::move(cmdRtBuffer);
    joint_handle_ptr_ = &joint_handle;

    return true;
  }

  void starting(const ros::Time &time) {
    if (!joint_handle_ptr_) {
      return;
    }

    double current_pos = joint_handle_ptr_->getPosition();
    double current_vel = joint_handle_ptr_->getVelocity();
    (*joint_handle_ptr_).setCommand(current_pos,current_vel,10,2,0.);
  }

  void stopping(const ros::Time &time) {}

  void updateCommand(const ros::Time &time, const ros::Duration &period) {
    // Update PIDs
    double des_position = (CmdBuff_->readFromRT())->des_pos;
    double des_velocity = (CmdBuff_->readFromRT())->des_vel;
    double des_kp = (CmdBuff_->readFromRT())->des_kp;
    double des_kd = (CmdBuff_->readFromRT())->des_kd;
    double des_eff = (CmdBuff_->readFromRT())->des_eff;

    (*joint_handle_ptr_).setCommand(des_position,des_velocity,des_kp,des_kd,des_eff);
  }

private:
  hardware_interface::HybridJointHandle *joint_handle_ptr_;
  std::shared_ptr<CmdBuff> CmdBuff_;
};

} // namespace gripper_controller
