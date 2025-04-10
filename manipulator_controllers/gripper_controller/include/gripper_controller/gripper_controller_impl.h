#include <memory>

#pragma once

namespace gripper_controller
{

template <class HardwareInterface>
GripperController<HardwareInterface>::GripperController()
{
  command_struct_.position_ = 0.0;
  command_struct_.max_effort_ = 0.0;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::starting(const ros::Time& time)
{
  double current_position = joint_.getPosition();

  target_position_ = current_position;
  target_effort_ = max_effort_;

  state_ = GripperState::IDLE;
  previous_state_ = GripperState::IDLE;
  is_stalled_ = false;
  stall_condition_active_ = false;
  self_test_active_ = false;

  if (verbose_) {
    ROS_INFO_STREAM_NAMED(name_, "Starting gripper controller at position " << current_position);
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::stopping(const ros::Time& time)
{
  if (verbose_) {
    ROS_INFO_STREAM_NAMED(name_, "Stopping gripper controller");
  }
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::init(hardware_interface::RobotHW* robot_hw,
                                                ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  try {
    controller_nh_ = controller_nh;
    name_ = controller_nh_.getNamespace();

    std::string joint_name;
    if (!controller_nh_.getParam("joint", joint_name)) {
      ROS_ERROR_STREAM_NAMED(name_, "No joint given in namespace: " << controller_nh_.getNamespace());
      return false;
    }

    auto* hw = robot_hw->get<HardwareInterface>();
    if (!hw) {
      ROS_ERROR_STREAM_NAMED(name_, "Could not get hardware interface");
      return false;
    }

    try {
      joint_ = hw->getHandle(joint_name);
    }
    catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM_NAMED(name_, "Exception getting joint handle: " << ex.what());
      return false;
    }

    hw_iface_adapter_.init(joint_, controller_nh_);

    if (!loadUrdf(root_nh)) {
      ROS_ERROR_STREAM_NAMED(name_, "Failed to load URDF");
      return false;
    }

    controller_nh_.param("verbose", verbose_, false);
    controller_nh_.param("position_tolerance", position_tolerance_, 0.01);
    controller_nh_.param("stalled_velocity", stalled_velocity_, 0.001);
    controller_nh_.param("stalled_force", stalled_force_, 0.5 * max_effort_);
    controller_nh_.param("stall_timeout", stall_timeout_, 0.5);
    controller_nh_.param<double>("release_offset", release_offset_, 0.01);

    dyn_reconfig_server_ = std::make_unique<dynamic_reconfigure::Server<GripperControllerConfig>>(controller_nh_);
    dynamic_reconfigure::Server<GripperControllerConfig>::CallbackType cb =
        boost::bind(&GripperController::dynamicReconfigureCallback, this, _1, _2);
    dyn_reconfig_server_->setCallback(cb);

    command_struct_.position_ = joint_.getPosition();
    command_struct_.max_effort_ = max_effort_;

    target_position_ = joint_.getPosition();
    target_effort_ = max_effort_;

    command_subscriber_ = controller_nh_.subscribe<std_msgs::Float64>(
        "command", 1, &GripperController::commandCB, this);

    gripper_command_server_ = controller_nh_.advertiseService(
        "gripper_command", &GripperController::handleGripperCommandService, this);

    status_pub_ = std::make_shared<realtime_tools::RealtimePublisher<manipulator_msgs::GripperStatus>>(
        controller_nh_, "status", 10);

    if (verbose_) {
      ROS_INFO_STREAM_NAMED(name_, "Successfully initialized gripper controller");
      ROS_INFO_STREAM_NAMED(name_, "Joint '" << joint_name << "' configuration:");
      ROS_INFO_STREAM_NAMED(name_, "- Position limits: [" << min_position_ << ", " << max_position_ << "]");
      ROS_INFO_STREAM_NAMED(name_, "- Maximum effort: " << max_effort_);
      ROS_INFO_STREAM_NAMED(name_, "- Maximum velocity: " << max_velocity_);
      ROS_INFO_STREAM_NAMED(name_, "- Position tolerance: " << position_tolerance_);
      ROS_INFO_STREAM_NAMED(name_, "- Stalled velocity threshold: " << stalled_velocity_);
      ROS_INFO_STREAM_NAMED(name_, "- Stalled force threshold: " << stalled_force_);
      ROS_INFO_STREAM_NAMED(name_, "- Stall timeout: " << stall_timeout_ << "s");
      ROS_INFO_STREAM_NAMED(name_, "- Release offset: " << release_offset_);
      ROS_INFO_STREAM_NAMED(name_, "Gripper command service started. Available commands: open, close, self_test, grip, release, anti_stall");
    }

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM_NAMED(name_, "Exception during init: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR_STREAM_NAMED(name_, "Unknown exception during init");
    return false;
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
  case GripperState::MOVING:
    handleMovingState();
    break;

  case GripperState::ERROR:
    handleErrorState();
    break;

  case GripperState::HOLDING:
    handleHoldingState();
    break;

  case GripperState::IDLE:
    handleIdleState();
    break;

  case GripperState::SELF_TEST:
    handleSelfTestState(time, period);
    break;
  }

  publishStatus(time);
  hw_iface_adapter_.updateCommand(period, target_position_, target_effort_);
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::loadUrdf(ros::NodeHandle& root_nh) {
  try {
    if (urdf_model_ == nullptr) {
      urdf_model_ = std::make_shared<urdf::Model>();
    }

    if (!root_nh.getParam("robot_description", urdf_string_)) {
      ROS_WARN("Failed to get robot_description parameter");
      return false;
    }

    if (urdf_string_.empty()) {
      ROS_WARN("Empty URDF string");
      return false;
    }

    if (!urdf_model_->initString(urdf_string_)) {
      ROS_WARN("Failed to parse URDF");
      return false;
    }

    urdf::JointConstSharedPtr joint = urdf_model_->getJoint(joint_.getName());
    if (!joint) {
      ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_.getName() << "' in URDF");
      return false;
    }

    // 检查关节类型
    if (joint->type != urdf::Joint::PRISMATIC && joint->type != urdf::Joint::REVOLUTE) {
      ROS_ERROR_STREAM_NAMED(name_, "Joint '" << joint_.getName() << "' is not a prismatic or revolute joint");
      return false;
    }

    // 检查是否有限制定义
    if (!joint->limits) {
      ROS_ERROR_STREAM_NAMED(name_, "Joint '" << joint_.getName() << "' has no limits defined");
      return false;
    }

    // 设置限制参数
    max_position_ = joint->limits->upper;
    min_position_ = joint->limits->lower;
    max_effort_ = joint->limits->effort;
    max_velocity_ = joint->limits->velocity;

    // 验证限制参数的有效性
    if (max_position_ <= min_position_) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid position limits for joint '" << joint_.getName()
                                                                          << "': upper limit (" << max_position_
                                                                          << ") <= lower limit (" << min_position_ << ")");
      return false;
    }

    if (max_effort_ <= 0.0) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid effort limit for joint '" << joint_.getName()
                                                                       << "': " << max_effort_);
      return false;
    }

    if (max_velocity_ <= 0.0) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid velocity limit for joint '" << joint_.getName()
                                                                         << "': " << max_velocity_);
      return false;
    }
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("Exception in loadUrdf: " << e.what());
    return false;
  } catch (...) {
    ROS_ERROR("Unknown exception in loadUrdf");
    return false;
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::setCommand(double position, double effort)
{
  // 确保位置在有效范围内
  if (position < min_position_) {
    position = min_position_;
  } else if (position > max_position_) {
    position = max_position_;
  }

  // 确保力矩在有效范围内
  if (effort <= 0.0) {
    effort = max_effort_;
  } else if (effort > max_effort_) {
    effort = max_effort_;
  }

  // 更新目标
  target_position_ = position;
  target_effort_ = effort;

  // 更新命令结构
  command_struct_.position_ = position;
  command_struct_.max_effort_ = effort;

  if (verbose_) {
    ROS_DEBUG_STREAM_NAMED(name_, "New command: position=" << position << ", effort=" << effort);
  }
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::isAtPosition(double target_position) const
{
  return std::abs(joint_.getPosition() - target_position) < position_tolerance_;
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::isForceExceeded(double max_force) const
{
  return std::abs(joint_.getEffort()) >= max_force;
}

//  <-------------------------State Machine Start--------------------------->
template <class HardwareInterface>
void GripperController<HardwareInterface>::handleHoldingState()
{
  // 检查是否仍然保持在位置
  if (!isAtPosition(target_position_)) {
    transitionTo(GripperState::MOVING);
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleIdleState()
{
  // 空闲状态下不执行任何操作
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleMovingState()
{
  if (isAtPosition(target_position_)) {
    transitionTo(GripperState::HOLDING);
    return;
  }

  if (isInstantStalled()) {
    if (isForceExceeded(target_effort_)) {
      transitionTo(GripperState::HOLDING);
    } else {
      transitionTo(GripperState::ERROR);
    }
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleErrorState()
{
  // 错误状态下不执行任何操作
}

// For self-test
template <class HardwareInterface>
void GripperController<HardwareInterface>::triggerSelfTest()
{
  if (state_ != GripperState::IDLE) {
    ROS_WARN_STREAM_NAMED(name_, "Cannot start self-test: gripper is not in IDLE state");
    return;
  }

  startSelfTest();
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::startSelfTest()
{
  ROS_INFO_STREAM_NAMED(name_, "Starting gripper self-test mode");

  self_test_active_ = true;
  self_test_cycle_count_ = 0;
  self_test_speed_factor_ = 1.0;
  self_test_phase_ = SelfTestPhase::OPENING;
  self_test_start_time_ = ros::Time::now();
  self_test_last_action_time_ = ros::Time::now();

  transitionTo(GripperState::SELF_TEST);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleSelfTestState(const ros::Time& time, const ros::Duration& period)
{
  if (!self_test_active_) {
    transitionTo(GripperState::IDLE);
    return;
  }

  if (self_test_cycle_count_ >= self_test_max_cycles_) {
    ROS_INFO_STREAM_NAMED(name_, "Self-test completed successfully after "
                                     << self_test_cycle_count_ << " cycles");
    self_test_active_ = false;
    transitionTo(GripperState::IDLE);
    return;
  }

  self_test_speed_factor_ = 1.0 + (self_test_cycle_count_ * 0.5);

  double current_position = joint_.getPosition();

  switch (self_test_phase_) {
  case SelfTestPhase::OPENING:
    target_position_ = max_position_;
    target_effort_ = max_effort_ * 0.7;

    if (isAtPosition(max_position_)) {
      ROS_INFO_STREAM_NAMED(name_, "Self-test cycle " << (self_test_cycle_count_ + 1)
                                                      << ": Gripper fully opened");
      self_test_phase_ = SelfTestPhase::CLOSING;
      self_test_last_action_time_ = time;
    }
    break;

  case SelfTestPhase::CLOSING:
    target_position_ = min_position_;
    target_effort_ = max_effort_ * 0.7;

    if (isAtPosition(min_position_)) {
      ROS_INFO_STREAM_NAMED(name_, "Self-test cycle " << (self_test_cycle_count_ + 1)
                                                      << ": Gripper fully closed");
      self_test_phase_ = SelfTestPhase::OPENING;
      self_test_cycle_count_++;
      self_test_last_action_time_ = time;
    }
    break;

  case SelfTestPhase::COMPLETED:
    self_test_active_ = false;
    transitionTo(GripperState::IDLE);
    break;
  }

  // 应用速度因子 - 通过调整PID控制器的参数来实现
  // 注意：这需要硬件接口适配器支持动态调整PID参数
  //  hw_iface_adapter_.setSpeedFactor(self_test_speed_factor_);
}

// Machine Tools
template <class HardwareInterface>
void GripperController<HardwareInterface>::transitionTo(GripperState new_state)
{
  if (state_ == new_state) return;

  previous_state_ = state_;
  state_ = new_state;

  if (verbose_)
    ROS_INFO_STREAM_NAMED(name_, "State transition: "
                                     << stateToString(previous_state_) << " -> " << stateToString(state_));
}

template <class HardwareInterface>
std::string GripperController<HardwareInterface>::stateToString(GripperState state)
{
  switch (state)
  {
  case GripperState::IDLE:      return "IDLE";
  case GripperState::MOVING:    return "MOVING";
  case GripperState::HOLDING:   return "HOLDING";
  case GripperState::ERROR:     return "ERROR";
  case GripperState::SELF_TEST: return "SELF_TEST";
  default:                      return "UNKNOWN";
  }
}

//  <-------------------------State Machine End--------------------------->

// For stall check
template <class HardwareInterface>
bool GripperController<HardwareInterface>::isInstantStalled() const
{
  return (std::abs(joint_.getVelocity()) < stalled_velocity_ &&
          std::abs(joint_.getEffort()) > stalled_force_);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::updateStallDetection(const ros::Time& current_time)
{
  bool current_condition = isInstantStalled();

  if (current_condition) {
    if (!stall_condition_active_) {
      stall_condition_active_ = true;
      stall_condition_met_time_ = current_time;
    }
  } else {
    stall_condition_active_ = false;
    stall_condition_met_time_ = ros::Time(0);
  }
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::isStalled(const ros::Time& current_time)
{
  updateStallDetection(current_time);

  if (stall_condition_active_ &&
      !stall_condition_met_time_.isZero() &&
      (current_time - stall_condition_met_time_).toSec() >= stall_timeout_) {
    return true;
  }

  return false;
}

// For pub
template <class HardwareInterface>
void GripperController<HardwareInterface>::publishStatus(const ros::Time& current_time)
{
  // 检查是否可以发布
  if (status_pub_ && status_pub_->trylock()) {
    auto& msg = status_pub_->msg_;

    // 填充消息头
    msg.header.stamp = current_time;
    msg.header.frame_id = joint_.getName();

    // 填充基本状态
    msg.state = stateToString(state_);
    msg.position = joint_.getPosition();
    msg.velocity = joint_.getVelocity();
    msg.effort = joint_.getEffort();

    // 填充目标信息
    msg.target_position = target_position_;
    msg.target_effort = target_effort_;

    // 填充堵转信息
    is_stalled_ = isStalled(current_time);
    msg.is_stalled = is_stalled_;
    msg.is_instant_stalled = isInstantStalled();

    msg.command = current_command_;

    // 发布消息
    status_pub_->unlockAndPublish();
  }
}

// For Callback
template <class HardwareInterface>
void GripperController<HardwareInterface>::dynamicReconfigureCallback(
    gripper_controller::GripperControllerConfig& config, uint32_t /*level*/)
{
  position_tolerance_ = config.position_tolerance;
  stalled_velocity_ = config.stalled_velocity_threshold;
  stalled_force_ = config.stalled_effort_threshold;
  stall_timeout_ = config.stall_timeout;

  release_offset_ = config.release_offset;

  if (verbose_) {
    ROS_INFO_STREAM_NAMED(name_, "Gripper controller parameters updated:");
    ROS_INFO_STREAM_NAMED(name_, "  Position tolerance: " << position_tolerance_);
    ROS_INFO_STREAM_NAMED(name_, "  Stall detection: velocity < " << stalled_velocity_
                                                                  << ", effort > " << stalled_force_
                                                                  << ", timeout: " << stall_timeout_ << "s");
    ROS_INFO_STREAM_NAMED(name_, "  Release offset: " << release_offset_);
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::commandCB(const std_msgs::Float64ConstPtr& msg)
{
  double position = msg->data;

  position = std::min(std::max(position, min_position_), max_position_);

  target_position_ = position;

  previous_state_ = state_;
  state_ = GripperState::MOVING;

  if (verbose_) {
    ROS_INFO_STREAM_NAMED(name_, "Received command: position=" << position);
  }
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::handleGripperCommandService(
    manipulator_msgs::GripperCommand::Request& req,
    manipulator_msgs::GripperCommand::Response& res)
{
  std::string command = req.command;
  double max_effort = req.max_effort > 0 ? req.max_effort : max_effort_;

  // 记录当前命令
  current_command_ = command;

  if (command == "open") {
    target_position_ = max_position_;
    target_effort_ = max_effort;
    transitionTo(GripperState::MOVING);

    res.success = true;
    res.message = "Opening gripper to position " + std::to_string(max_position_);
  }
  else if (command == "close") {
    target_position_ = min_position_;
    target_effort_ = max_effort;
    transitionTo(GripperState::MOVING);

    res.success = true;
    res.message = "Closing gripper";
  }
  else if (command == "self_test") {
    if (state_ != GripperState::IDLE && state_ != GripperState::HOLDING) {
      res.success = false;
      res.message = "Cannot start self-test: gripper is not in IDLE or HOLDING state";
    } else {
      triggerSelfTest();
      res.success = true;
      res.message = "Self-test started";
    }
  }
  else {
    res.success = false;
    res.message = "Unknown command: " + command +
                  ". Available commands: open, close, self_test";
  }

  return true;
}
} // namespace gripper_controller
