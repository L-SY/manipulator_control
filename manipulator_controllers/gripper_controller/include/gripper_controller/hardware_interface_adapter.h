#pragma once

#include <cassert>
#include <string>
#include <limits>
#include <cmath>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

// 包含自定义的混合接口
#include <manipulator_common/interface/hardware_interface/HybridJointInterface.h>

namespace gripper_controller
{

/**
 * \brief 硬件接口适配器的基类模板
 *
 * 该类为不同类型的硬件接口提供统一的接口
 */
template <class HardwareInterface>
class HardwareInterfaceAdapter
{
public:
  virtual ~HardwareInterfaceAdapter() = default;

  /**
   * \brief 初始化适配器
   * \param joint_handle 关节句柄
   * \param nh 控制器的节点句柄
   */
  virtual bool init(HardwareInterface* hardwareInterface, ros::NodeHandle& nh) = 0;

  /**
   * \brief 更新命令
   * \param period 控制周期
   * \param position 目标位置
   * \param max_effort 最大力
   */
  virtual void updateCommand(const ros::Duration& period, double position, double max_effort) = 0;

  /**
   * \brief 获取当前关节位置
   * \return 当前关节位置（弧度）
   */
  virtual double getPosition() = 0;

  /**
   * \brief 获取当前关节速度
   * \return 当前关节速度（弧度/秒）
   */
  virtual double getVelocity() = 0;

  /**
   * \brief 获取当前关节力矩
   * \return 当前关节力矩（牛米）
   */
  virtual double getEffort() = 0;
};

/**
 * \brief 位置接口适配器
 *
 * 用于位置控制接口，支持速度控制的位置插值
 */
template <>
class HardwareInterfaceAdapter<hardware_interface::PositionJointInterface>
{
public:
  HardwareInterfaceAdapter() : position_command_(0.0), velocity_limit_(0.0), has_velocity_limit_(false) {}

  virtual ~HardwareInterfaceAdapter() = default;


  bool init(hardware_interface::PositionJointInterface* positionJointInterface, ros::NodeHandle& nh)
  {
    std::string joint_name;
    nh.getParam("joint", joint_name);
    joint_handle_ = positionJointInterface->getHandle(joint_name);

    // 加载参数
    nh.param<double>("velocity_limit", velocity_limit_, 0.0);
    has_velocity_limit_ = velocity_limit_ > 0.0;

    if (has_velocity_limit_)
      ROS_INFO_STREAM("Velocity limit enabled: " << velocity_limit_ << " rad/s");

    // 初始化命令为当前位置
    position_command_ = joint_handle_.getPosition();

    return true;
  }

  void updateCommand(const ros::Duration& period, double position, double /*max_effort*/)
  {
    // 获取当前位置
    double current_position = joint_handle_.getPosition();

    // 如果启用了速度限制，则进行位置插值
    if (has_velocity_limit_)
    {
      // 计算位置差
      double position_error = position - current_position;

      // 计算这个周期内最大可移动距离
      double max_step = velocity_limit_ * period.toSec();

      // 限制位置变化
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
      // 不限制速度，直接设置目标位置
      position_command_ = position;
    }

    // 发送命令到硬件
    joint_handle_.setCommand(position_command_);
  }

  /**
   * \brief 获取当前关节位置
   * \return 当前关节位置（弧度）
   */
  double getPosition()
  {
    return joint_handle_.getPosition();
  }

  /**
   * \brief 获取当前关节速度
   * \return 当前关节速度（弧度/秒）
   */
  double getVelocity()
  {
    return joint_handle_.getVelocity();
  }

  /**
   * \brief 获取当前关节力矩
   * \return 当前关节力矩（牛米）
   */
  double getEffort()
  {
    return joint_handle_.getEffort();
  }

private:
  hardware_interface::JointHandle joint_handle_;
  double position_command_;
  double velocity_limit_;
  bool has_velocity_limit_;
};

/**
 * \brief 力矩接口适配器
 *
 * 用于力矩控制接口，使用PID控制器实现位置控制
 */
template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface>
{
public:
  HardwareInterfaceAdapter() : position_command_(0.0), velocity_limit_(0.0), has_velocity_limit_(false) {}

  virtual ~HardwareInterfaceAdapter() = default;

  bool init(hardware_interface::EffortJointInterface* effortJointInterface, ros::NodeHandle& nh)
  {
    std::string joint_name;
    nh.getParam("joint", joint_name);
    joint_handle_ = effortJointInterface->getHandle(joint_name);

    // 初始化PID控制器
    control_toolbox::Pid pid;
    if (!pid.init(ros::NodeHandle(nh, "pid")))
    {
      ROS_ERROR("Failed to initialize PID controller");
      return false;
    }
    pid_controller_ = pid;

    // 加载参数
    nh.param<double>("velocity_limit", velocity_limit_, 0.0);
    has_velocity_limit_ = velocity_limit_ > 0.0;

    if (has_velocity_limit_)
      ROS_INFO_STREAM("Velocity limit enabled: " << velocity_limit_ << " rad/s");

    // 初始化命令为当前位置
    position_command_ = joint_handle_.getPosition();

    return true;
  }

  void updateCommand(const ros::Duration& period, double position, double max_effort)
  {
    // 获取当前位置
    double current_position = joint_handle_.getPosition();

    // 如果启用了速度限制，则进行位置插值
    if (has_velocity_limit_)
    {
      // 计算位置差
      double position_error = position - current_position;

      // 计算这个周期内最大可移动距离
      double max_step = velocity_limit_ * period.toSec();

      // 限制位置变化
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
      // 不限制速度，直接设置目标位置
      position_command_ = position;
    }

    // 计算位置误差
    double error = position_command_ - current_position;

    // 使用PID控制器计算输出力矩
    double commanded_effort = pid_controller_.computeCommand(error, period);

    // 限制输出力矩
    commanded_effort = std::max(-max_effort, std::min(commanded_effort, max_effort));

    // 发送命令到硬件
    joint_handle_.setCommand(commanded_effort);
  }

  /**
   * \brief 获取当前关节位置
   * \return 当前关节位置（弧度）
   */
  double getPosition()
  {
    return joint_handle_.getPosition();
  }

  /**
   * \brief 获取当前关节速度
   * \return 当前关节速度（弧度/秒）
   */
  double getVelocity()
  {
    return joint_handle_.getVelocity();
  }

  /**
   * \brief 获取当前关节力矩
   * \return 当前关节力矩（牛米）
   */
  double getEffort()
  {
    return joint_handle_.getEffort();
  }

private:
  hardware_interface::JointHandle joint_handle_;
  control_toolbox::Pid pid_controller_;
  double position_command_;
  double velocity_limit_;
  bool has_velocity_limit_;
};

/**
 * \brief 混合接口适配器
 *
 * 用于混合控制接口，支持位置、速度和力的混合控制
 */
template <>
class HardwareInterfaceAdapter<hardware_interface::HybridJointInterface>
{
public:
  HardwareInterfaceAdapter()
      : position_command_(0.0)
        , velocity_command_(0.0)
        , kp_(100.0)
        , kd_(1.0)
        , ff_(0.0)
        , velocity_limit_(0.0)
        , has_velocity_limit_(false) {}

  virtual ~HardwareInterfaceAdapter() = default;

  bool init(hardware_interface::HybridJointInterface* hybridJointInterface, ros::NodeHandle& nh)
  {
    std::string joint_name;
    nh.getParam("joint", joint_name);
    hybrid_joint_handle_ = hybridJointInterface->getHandle(joint_name);

    // 加载参数
    nh.param<double>("kp", kp_, 100.0);
    nh.param<double>("kd", kd_, 1.0);
    nh.param<double>("velocity_limit", velocity_limit_, 0.0);
    has_velocity_limit_ = velocity_limit_ > 0.0;

    if (has_velocity_limit_)
      ROS_INFO_STREAM("Velocity limit enabled: " << velocity_limit_ << " rad/s");

    // 初始化命令为当前位置
    position_command_ = hybrid_joint_handle_.getPosition();
    velocity_command_ = 0.0;

    return true;
  }

  void updateCommand(const ros::Duration& period, double position, double max_effort)
  {
    // 获取当前位置
    double current_position = hybrid_joint_handle_.getPosition();

    // 如果启用了速度限制，则进行位置插值
    if (has_velocity_limit_)
    {
      // 计算位置差
      double position_error = position - current_position;

      // 计算这个周期内最大可移动距离
      double max_step = velocity_limit_ * period.toSec();

      // 限制位置变化
      if (std::abs(position_error) > max_step)
      {
        position_command_ = current_position + (position_error > 0 ? max_step : -max_step);
        velocity_command_ = velocity_limit_ * (position_error > 0 ? 1.0 : -1.0);
      }
      else
      {
        position_command_ = position;
        // 计算速度命令 (可以根据位置误差比例计算)
        double ratio = std::min(1.0, std::abs(position_error) / max_step);
        velocity_command_ = velocity_limit_ * ratio * (position_error > 0 ? 1.0 : -1.0);
      }
    }
    else
    {
      // 不限制速度，直接设置目标位置
      position_command_ = position;
      velocity_command_ = 0.0; // 或者可以设置为一个默认值
    }

    // 计算前馈力矩 (这里简单地使用最大力的一部分)
    ff_ = max_effort * 0.1; // 使用最大力的10%作为前馈

    // 发送命令到混合接口
    hybrid_joint_handle_.setCommand(position_command_, velocity_command_, kp_, kd_, ff_);
  }

  /**
   * \brief 获取当前关节位置
   * \return 当前关节位置（弧度）
   */
  double getPosition()
  {
    return hybrid_joint_handle_.getPosition();
  }

  /**
   * \brief 获取当前关节速度
   * \return 当前关节速度（弧度/秒）
   */
  double getVelocity()
  {
    return hybrid_joint_handle_.getVelocity();
  }

  /**
   * \brief 获取当前关节力矩
   * \return 当前关节力矩（牛米）
   */
  double getEffort()
  {
    return hybrid_joint_handle_.getEffort();
  }

private:
  hardware_interface::HybridJointHandle hybrid_joint_handle_;
  double position_command_;
  double velocity_command_;
  double kp_;
  double kd_;
  double ff_;
  double velocity_limit_;
  bool has_velocity_limit_;
};

} // namespace gripper_controller
