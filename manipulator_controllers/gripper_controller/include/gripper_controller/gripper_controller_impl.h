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
  setCommand(joint_.getPosition(), stalled_force_);
  state_ = GripperState::IDLE;
  previous_state_ = GripperState::IDLE;
  is_stalled_ = false;
  constant_force_ = false;
  stall_condition_active_ = false;
  self_test_active_ = false;
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
    controller_nh_.param("stalled_force", stalled_force_, 50.0);
    controller_nh_.param("stall_timeout", stall_timeout_, 0.5);
    controller_nh_.param<double>("release_offset", release_offset_, 0.01);

    dyn_reconfig_server_ = std::make_unique<dynamic_reconfigure::Server<GripperControllerConfig>>(controller_nh_);
    dynamic_reconfigure::Server<GripperControllerConfig>::CallbackType cb =
        boost::bind(&GripperController::dynamicReconfigureCallback, this, _1, _2);
    dyn_reconfig_server_->setCallback(cb);

    command_struct_.position_ = joint_.getPosition();
    command_struct_.max_effort_ = max_effort_;

    setCommand(joint_.getPosition(), stalled_force_);
    command_subscriber_ = controller_nh_.subscribe<std_msgs::Float64>(
        "command", 1, &GripperController::commandCB, this);
    normalized_command_subscriber_ = controller_nh_.subscribe<std_msgs::Float64>(
        "normalized_command", 1, &GripperController::normalizedCommandCB, this);
    gripper_command_server_ = controller_nh_.advertiseService(
        "gripper_command", &GripperController::handleGripperCommandService, this);

    status_pub_ = std::make_shared<realtime_tools::RealtimePublisher<manipulator_msgs::GripperStatus>>(
        controller_nh_, "status", 10);

    if (verbose_) {
      ROS_INFO_STREAM_NAMED(name_, "Successfully initialized gripper controller");
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
  hw_iface_adapter_.updateCommand(period, target_position_, target_effort_, constant_force_);
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::loadUrdf(ros::NodeHandle& root_nh) {
  try {
    if (!urdf_model_) urdf_model_ = std::make_shared<urdf::Model>();

    if (!root_nh.getParam("robot_description", urdf_string_) || urdf_string_.empty() || !urdf_model_->initString(urdf_string_)) {
      ROS_WARN_STREAM_NAMED(name_, "Failed to load or parse 'robot_description'");
      return false;
    }

    auto joint = urdf_model_->getJoint(joint_.getName());
    if (!joint || (joint->type != urdf::Joint::PRISMATIC && joint->type != urdf::Joint::REVOLUTE) || !joint->limits) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid joint '" << joint_.getName() << "' in URDF");
      return false;
    }

    max_position_ = joint->limits->upper;
    min_position_ = joint->limits->lower;
    max_effort_ = joint->limits->effort;
    max_velocity_ = joint->limits->velocity;

    if (max_position_ <= min_position_ || max_effort_ <= 0.0 || max_velocity_ <= 0.0) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid limits for joint '" << joint_.getName() << "'");
      return false;
    }
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM_NAMED(name_, "Exception in loadUrdf: " << e.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM_NAMED(name_, "Unknown exception in loadUrdf");
    return false;
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::setCommand(double position, double effort) {
  position = std::clamp(position, min_position_, max_position_);
  effort = (effort > 0.0) ? std::min(effort, max_effort_) : max_effort_;

  target_position_ = position;
  target_effort_ = effort;
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

//  <-------------------------State Machine Start--------------------------->
template <class HardwareInterface>
void GripperController<HardwareInterface>::handleHoldingState()
{
  constant_force_ = true;
  setCommand(joint_.getPosition(), stalled_force_ * 0.25);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleIdleState()
{
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleMovingState()
{
  if (isAtPosition(target_position_)) {
    transitionTo(GripperState::IDLE);
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleErrorState()
{
}

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
  self_test_active_ = true;
  self_test_cycle_count_ = 0;
  self_test_speed_factor_ = 1.0;
  self_test_phase_ = SelfTestPhase::OPENING;

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
    self_test_active_ = false;
    transitionTo(GripperState::IDLE);
    return;
  }

  switch (self_test_phase_) {
  case SelfTestPhase::OPENING:
    setCommand(max_position_, stalled_force_);

    if (isAtPosition(max_position_)) {
      self_test_phase_ = SelfTestPhase::CLOSING;
    }
    break;

  case SelfTestPhase::CLOSING:
    setCommand(min_position_, stalled_force_);

    if (isAtPosition(min_position_)) {
      self_test_phase_ = SelfTestPhase::OPENING;
      self_test_cycle_count_++;
    }
    break;

  case SelfTestPhase::COMPLETED:
    self_test_active_ = false;
    transitionTo(GripperState::IDLE);
    break;
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::transitionTo(GripperState new_state)
{
  if (state_ == new_state) return;
  previous_state_ = state_;

  state_ = new_state;

  if(state_ != GripperState::HOLDING)
    constant_force_ = false;

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
  if (state_ == GripperState::HOLDING)
    return (std::abs(joint_.getVelocity()) < stalled_velocity_ &&
            std::abs(joint_.getEffort()) > 5);
  else
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
    if (self_test_active_)
      transitionTo(GripperState::ERROR);
    else if (state_ == GripperState::MOVING)
      transitionTo(GripperState::HOLDING);
    return true;
  }
  else
  {
    if (state_ == GripperState::HOLDING) {
      if (!stall_cleared_timing_active_) {
        stall_cleared_time_ = current_time;
        stall_cleared_timing_active_ = true;
      }
      else if ((current_time - stall_cleared_time_).toSec() >= stall_timeout_ / 2.0) {
        transitionTo(GripperState::IDLE);
        stall_cleared_timing_active_ = false;
      }
    }
    else {
      stall_cleared_timing_active_ = false;
    }
  }

  return false;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::publishStatus(const ros::Time& current_time)
{
  if (status_pub_ && status_pub_->trylock()) {
    auto& msg = status_pub_->msg_;

    msg.header.stamp = current_time;
    msg.header.frame_id = joint_.getName();

    msg.state = stateToString(state_);
    msg.current_state = static_cast<int8_t>(state_);
    msg.previous_state = static_cast<int8_t>(previous_state_);
    msg.position = joint_.getPosition();
    msg.velocity = joint_.getVelocity();
    msg.effort = joint_.getEffort();

    msg.target_position = target_position_;
    msg.target_effort = target_effort_;

    is_stalled_ = isStalled(current_time);
    msg.is_stalled = is_stalled_;
    msg.is_instant_stalled = isInstantStalled();

    msg.command = current_command_;

    status_pub_->unlockAndPublish();
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::dynamicReconfigureCallback(
    gripper_controller::GripperControllerConfig& config, uint32_t /*level*/)
{
  position_tolerance_ = config.position_tolerance;
  stalled_velocity_ = config.stalled_velocity_threshold;
  stalled_force_ = config.stalled_effort_threshold;
  stall_timeout_ = config.stall_timeout;

  release_offset_ = config.release_offset;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::commandCB(const std_msgs::Float64ConstPtr& msg) {
  double target_position = std::clamp(msg->data, min_position_, max_position_);

  if (((state_ == GripperState::HOLDING && target_position > target_position_) ||
      state_ == GripperState::IDLE ||state_ == GripperState::MOVING)  && target_position != target_position_)
  {
    setCommand(target_position, stalled_force_);
    transitionTo(GripperState::MOVING);
  }

  if (verbose_) ROS_INFO_STREAM_NAMED(name_, "Received command: position=" << target_position_);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::normalizedCommandCB(const std_msgs::Float64ConstPtr& msg) {
  double normalized_cmd = std::clamp(msg->data, -1.0, 1.0);

  //output = min + (input - inputMin) * (max - min) / (inputMax - inputMin)
  double target_position = min_position_ + (normalized_cmd - (-1.0)) * (max_position_ - min_position_) / (1.0 - (-1.0));

  if (((state_ == GripperState::HOLDING && target_position > target_position_) ||
       state_ == GripperState::IDLE ||state_ == GripperState::MOVING)  && target_position != target_position_)
  {
    setCommand(target_position, stalled_force_);
    transitionTo(GripperState::MOVING);
  }

  if (verbose_) ROS_INFO_STREAM_NAMED(name_, "Received normalized command: " << normalized_cmd
                                                                 << " -> position=" << target_position);
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::handleGripperCommandService(
    manipulator_msgs::GripperCommand::Request& req,
    manipulator_msgs::GripperCommand::Response& res) {
  current_command_ = req.command;
  target_effort_ = req.max_effort > 0 ? req.max_effort : max_effort_;

  if (req.command == "open") {
    setCommand(max_position_, stalled_force_);
    transitionTo(GripperState::MOVING);
    res.success = true;
    res.message = "Opening gripper to position " + std::to_string(max_position_);
  } else if (req.command == "close") {
    setCommand(min_position_, stalled_force_);
    transitionTo(GripperState::MOVING);
    res.success = true;
    res.message = "Closing gripper";
  } else if (req.command == "self_test") {
    if (state_ == GripperState::IDLE) {
      triggerSelfTest();
      res.success = true;
      res.message = "Self-test started";
    } else {
      res.success = false;
      res.message = "Cannot start self-test: gripper is not in IDLE or HOLDING state";
    }
  } else {
    res.success = false;
    res.message = "Unknown command: " + req.command + ". Available commands: open, close, self_test";
  }

  return true;
}

} // namespace gripper_controller