#pragma once

#include <cassert>
#include <string>
#include <limits>
#include <cmath>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <manipulator_common/interface/hardware_interface/HybridJointInterface.h>

namespace gripper_controller
{

template <class HardwareInterface>
class HardwareInterfaceAdapter
{
public:
  typedef typename HardwareInterface::ResourceHandleType HandleType;

  virtual ~HardwareInterfaceAdapter() = default;
  virtual bool init(const HandleType& handle, ros::NodeHandle& nh) = 0;
  virtual void updateCommand(const ros::Duration& period, double position, double max_effort) = 0;

private:
  HandleType joint_;
};

template <>
class HardwareInterfaceAdapter<hardware_interface::PositionJointInterface>
{
public:
  typedef hardware_interface::JointHandle HandleType;

  virtual ~HardwareInterfaceAdapter() = default;

  bool init(const HandleType& handle, ros::NodeHandle& nh)
  {
    joint_ = handle;

    nh.param<double>("velocity_limit", velocity_limit_, 0.0);
    has_velocity_limit_ = velocity_limit_ > 0.0;

    if (has_velocity_limit_)
      ROS_INFO_STREAM("Velocity limit enabled: " << velocity_limit_ << " rad/s");

    position_command_ = joint_.getPosition();

    return true;
  }

  void updateCommand(const ros::Duration& period, double position, double /*max_effort*/)
  {
    double current_position = joint_.getPosition();

    if (has_velocity_limit_)
    {
      double position_error = position - current_position;
      double max_step = velocity_limit_ * period.toSec();

      if (std::abs(position_error) > max_step)
      {
        position_command_ = current_position + (position_error > 0 ? max_step : -max_step);
      }
      else
      {
        position_command_ = position;
      }
    }
    else
    {
      position_command_ = position;
    }

    joint_.setCommand(position_command_);
  }

private:
  HandleType joint_;
  double position_command_{0.0};
  double velocity_limit_{0.0};
  bool has_velocity_limit_{false};
};


template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface>
{
public:
  typedef hardware_interface::JointHandle HandleType;

  virtual ~HardwareInterfaceAdapter() = default;

  bool init(const HandleType& handle, ros::NodeHandle& nh)
  {
    joint_ = handle;

    if (!pid_controller_.init(ros::NodeHandle(nh, "pid")))
    {
      ROS_ERROR("Failed to initialize PID controller");
      return false;
    }

    nh.param<double>("velocity_limit", velocity_limit_, 0.0);
    has_velocity_limit_ = velocity_limit_ > 0.0;

    if (has_velocity_limit_)
      ROS_INFO_STREAM("Velocity limit enabled: " << velocity_limit_ << " rad/s");

    position_command_ = joint_.getPosition();

    return true;
  }

  void updateCommand(const ros::Duration& period, double position, double max_effort)
  {
    double current_position = joint_.getPosition();

    if (has_velocity_limit_)
    {
      double position_error = position - current_position;
      double max_step = velocity_limit_ * period.toSec();

      if (std::abs(position_error) > max_step)
      {
        position_command_ = current_position + (position_error > 0 ? max_step : -max_step);
      }
      else
      {
        position_command_ = position;
      }
    }
    else
    {
      position_command_ = position;
    }

    double error = position_command_ - current_position;
    double commanded_effort = pid_controller_.computeCommand(error, period);
    commanded_effort = std::max(-max_effort, std::min(commanded_effort, max_effort));

    joint_.setCommand(commanded_effort);
  }

private:
  HandleType joint_;
  control_toolbox::Pid pid_controller_;
  double position_command_{0.0};
  double velocity_limit_{0.0};
  bool has_velocity_limit_{false};
};

template <>
class HardwareInterfaceAdapter<hardware_interface::HybridJointInterface>
{
public:
  typedef hardware_interface::HybridJointHandle HandleType;

  virtual ~HardwareInterfaceAdapter() = default;

  bool init(const HandleType& handle, ros::NodeHandle& nh)
  {
    joint_ = handle;

    nh.param<double>("kp", kp_, 100.0);
    nh.param<double>("kd", kd_, 1.0);
    nh.param<double>("velocity_limit", velocity_limit_, 0.0);
    has_velocity_limit_ = velocity_limit_ > 0.0;

    if (has_velocity_limit_)
      ROS_INFO_STREAM("Velocity limit enabled: " << velocity_limit_ << " rad/s");

    position_command_ = joint_.getPosition();
    velocity_command_ = 0.0;

    return true;
  }

  void updateCommand(const ros::Duration& period, double position, double max_effort)
  {
    double current_position = joint_.getPosition();

    if (has_velocity_limit_)
    {
      double position_error = position - current_position;
      double max_step = velocity_limit_ * period.toSec();

      if (std::abs(position_error) > max_step)
      {
        position_command_ = current_position + (position_error > 0 ? max_step : -max_step);
        velocity_command_ = velocity_limit_ * (position_error > 0 ? 1.0 : -1.0);
      }
      else
      {
        position_command_ = position;
        double ratio = std::min(1.0, std::abs(position_error) / max_step);
        velocity_command_ = velocity_limit_ * ratio * (position_error > 0 ? 1.0 : -1.0);
      }
    }
    else
    {
      position_command_ = position;
      velocity_command_ = 0.0;
    }

    ff_ = max_effort * 0.1;
    joint_.setCommand(position_command_, velocity_command_, kp_, kd_, ff_);
  }

private:
  HandleType joint_;
  double position_command_{0.0};
  double velocity_command_{0.0};
  double kp_{100.0};
  double kd_{1.0};
  double ff_{0.0};
  double velocity_limit_{0.0};
  bool has_velocity_limit_{false};
};

} // namespace gripper_controller
