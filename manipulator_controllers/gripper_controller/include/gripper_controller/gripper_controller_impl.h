#pragma once

namespace gripper_controller
{

template <class HardwareInterface>
GripperController<HardwareInterface>::GripperController()
    : state_(GripperState::IDLE)
      , previous_state_(GripperState::IDLE)
      , target_position_(0.0)
      , target_effort_(0.0)
{
  command_struct_.position_ = 0.0;
  command_struct_.max_effort_ = 0.0;
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::init(hardware_interface::RobotHW* robot_hw,
                                                ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  try {
    controller_nh_ = controller_nh;
    name_ = controller_nh_.getNamespace();

    // 获取关节名称
    std::string joint_name;
    if (!controller_nh_.getParam("joint", joint_name)) {
      ROS_ERROR_STREAM_NAMED(name_, "No joint given in namespace: " << controller_nh_.getNamespace());
      return false;
    }

    // 加载URDF并获取限制参数
    if (!loadUrdf(root_nh)) {
      ROS_ERROR_STREAM_NAMED(name_, "Failed to load URDF");
      return false;
    }

    // 从URDF中获取关节限制
    urdf::JointConstSharedPtr joint = urdf_model_->getJoint(joint_name);
    if (!joint) {
      ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_name << "' in URDF");
      return false;
    }

    // 检查关节类型
    if (joint->type != urdf::Joint::PRISMATIC && joint->type != urdf::Joint::REVOLUTE) {
      ROS_ERROR_STREAM_NAMED(name_, "Joint '" << joint_name << "' is not a prismatic or revolute joint");
      return false;
    }

    // 检查是否有限制定义
    if (!joint->limits) {
      ROS_ERROR_STREAM_NAMED(name_, "Joint '" << joint_name << "' has no limits defined");
      return false;
    }

    // 设置限制参数
    max_position_ = joint->limits->upper;
    min_position_ = joint->limits->lower;
    max_effort_ = joint->limits->effort;
    max_velocity_ = joint->limits->velocity;

    // 验证限制参数的有效性
    if (max_position_ <= min_position_) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid position limits for joint '" << joint_name
                                                                          << "': upper limit (" << max_position_
                                                                          << ") <= lower limit (" << min_position_ << ")");
      return false;
    }

    if (max_effort_ <= 0.0) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid effort limit for joint '" << joint_name
                                                                       << "': " << max_effort_);
      return false;
    }

    if (max_velocity_ <= 0.0) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid velocity limit for joint '" << joint_name
                                                                         << "': " << max_velocity_);
      return false;
    }

    // 加载其他控制器参数
    controller_nh_.param("verbose", verbose_, false);

    // 设置动态参数的默认值
    controller_nh_.param("position_tolerance", position_tolerance_, 0.01);
    controller_nh_.param("stalled_velocity", stalled_velocity_, 0.001);
    controller_nh_.param("stalled_force", stalled_force_, 0.1 * max_effort_); // 默认为最大力矩的10%

    // 验证动态参数的有效性
    if (position_tolerance_ <= 0.0) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid position tolerance: " << position_tolerance_);
      return false;
    }

    if (stalled_velocity_ <= 0.0) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid stalled velocity threshold: " << stalled_velocity_);
      return false;
    }

    if (stalled_force_ <= 0.0 || stalled_force_ > max_effort_) {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid stalled force threshold: " << stalled_force_);
      return false;
    }

    dyn_reconfig_server_.reset(new dynamic_reconfigure::Server<GripperControllerConfig>(controller_nh_));
    dynamic_reconfigure::Server<GripperControllerConfig>::CallbackType cb =
        boost::bind(&GripperController::dynamicReconfigureCallback, this, _1, _2);
    dyn_reconfig_server_->setCallback(cb);

    // 初始化硬件接口
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

    // 初始化硬件接口适配器
    hw_iface_adapter_.init(joint_, controller_nh_);

    // 初始化命令结构
    command_struct_.position_ = joint_.getPosition();
    command_struct_.max_effort_ = max_effort_;

    // 初始化目标位置和力矩
    target_position_ = joint_.getPosition();
    target_effort_ = max_effort_;

    // 初始化ROS接口
    command_subscriber_ = controller_nh_.subscribe<std_msgs::Float64>(
        "command", 1, &GripperController::commandCB, this);
    state_publisher_ = controller_nh_.advertise<std_msgs::String>("state", 1);
    stall_status_pub_ = controller_nh.advertise<manipulator_msgs::GripperStallStatus>("stall_status", 10);

    if (verbose_) {
      ROS_INFO_STREAM_NAMED(name_, "Successfully initialized gripper controller");
      ROS_INFO_STREAM_NAMED(name_, "Joint '" << joint_name << "' configuration:");
      ROS_INFO_STREAM_NAMED(name_, "- Position limits: [" << min_position_ << ", " << max_position_ << "]");
      ROS_INFO_STREAM_NAMED(name_, "- Maximum effort: " << max_effort_);
      ROS_INFO_STREAM_NAMED(name_, "- Maximum velocity: " << max_velocity_);
      ROS_INFO_STREAM_NAMED(name_, "- Position tolerance: " << position_tolerance_);
      ROS_INFO_STREAM_NAMED(name_, "- Stalled velocity threshold: " << stalled_velocity_);
      ROS_INFO_STREAM_NAMED(name_, "- Stalled force threshold: " << stalled_force_);
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
bool GripperController<HardwareInterface>::loadUrdf(ros::NodeHandle& root_nh)
{
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

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in loadUrdf: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in loadUrdf");
    return false;
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::dynamicReconfigureCallback(
    gripper_controller::GripperControllerConfig& config, uint32_t /*level*/)
{
  position_tolerance_ = config.position_tolerance;
  stalled_velocity_ = config.stalled_velocity;
  stalled_force_ = config.stalled_force;
  stall_timeout_ = config.stall_timeout;
}

// 检查瞬时堵转条件（原来的isStalled逻辑）
template <class HardwareInterface>
bool GripperController<HardwareInterface>::checkStallCondition() const
{
  return (std::abs(joint_.getVelocity()) < stalled_velocity_ &&
          std::abs(joint_.getEffort()) > stalled_force_);
}

// 更新堵转检测状态
template <class HardwareInterface>
void GripperController<HardwareInterface>::updateStallDetection(const ros::Time& current_time)
{
  bool current_condition = checkStallCondition();

  // 如果当前满足堵转条件
  if (current_condition) {
    // 如果是第一次满足条件，记录时间
    if (!stall_condition_active_) {
      stall_condition_active_ = true;
      stall_condition_met_time_ = current_time;
    }
    // 否则继续保持状态，时间不更新
  } else {
    // 不满足条件，重置状态
    stall_condition_active_ = false;
    stall_condition_met_time_ = ros::Time(0);
  }
}

// 新的带时间判断的isStalled方法
template <class HardwareInterface>
bool GripperController<HardwareInterface>::isStalled(const ros::Time& current_time)
{
  // 更新堵转检测状态
  updateStallDetection(current_time);

  // 如果当前满足堵转条件，且持续时间超过阈值
  if (stall_condition_active_ &&
      !stall_condition_met_time_.isZero() &&
      (current_time - stall_condition_met_time_).toSec() >= stall_timeout_) {
    return true;
  }

  return false;
}


template <class HardwareInterface>
void GripperController<HardwareInterface>::starting(const ros::Time& /*time*/)
{
  // 初始化状态
  state_ = GripperState::IDLE;
  previous_state_ = GripperState::IDLE;

  // 使用当前位置
  target_position_ = joint_.getPosition();
  target_effort_ = max_effort_;

  if (verbose_)
    ROS_INFO_STREAM_NAMED(name_, "Starting controller");
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::stopping(const ros::Time& /*time*/)
{
  if (verbose_)
    ROS_INFO_STREAM_NAMED(name_, "Stopping controller");
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
  }

  publishStallInformation(time);
  hw_iface_adapter_.updateCommand(period, target_position_, target_effort_);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleMovingState()
{
  if (isAtPosition(target_position_, position_tolerance_)) {
    transitionTo(GripperState::HOLDING);
    return;
  }

  if (checkStallCondition()) {
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

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleHoldingState()
{
  // 检查是否仍然保持在位置
  if (!isAtPosition(target_position_, position_tolerance_)) {
    transitionTo(GripperState::MOVING);
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleIdleState()
{
  // 空闲状态下不执行任何操作
}


template <class HardwareInterface>
void GripperController<HardwareInterface>::commandCB(const std_msgs::Float64::ConstPtr& msg)
{
  // 检查位置是否在有效范围内
  double target = msg->data;
  if (target < min_position_ || target > max_position_) {
    ROS_WARN_STREAM_NAMED(name_, "Position command out of range");
    return;
  }

  // 更新目标位置
  target_position_ = target;
  target_effort_ = max_effort_;

  // 更新命令结构
  command_struct_.position_ = target_position_;
  command_struct_.max_effort_ = target_effort_;

  // 切换到运动状态
  if (state_ == GripperState::IDLE || state_ == GripperState::HOLDING) {
    transitionTo(GripperState::MOVING);
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::transitionTo(GripperState new_state)
{
  if (state_ == new_state) return;

  previous_state_ = state_;
  state_ = new_state;

  if (verbose_)
    ROS_INFO_STREAM_NAMED(name_, "State transition: "
                                     << stateToString(previous_state_) << " -> " << stateToString(state_));

  publishState();
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::isAtPosition(double target_position, double tolerance) const
{
  return std::abs(joint_.getPosition() - target_position) < tolerance;
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::isForceExceeded(double max_force) const
{
  return std::abs(joint_.getEffort()) >= max_force;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::publishState()
{
  std_msgs::String state_msg;
  state_msg.data = stateToString(state_);
  state_publisher_.publish(state_msg);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::publishStallInformation(const ros::Time& current_time)
{
  // 检查当前堵转状态
  bool is_stalled = isStalled(current_time);

  // 创建堵转信息消息
  manipulator_msgs::GripperStallStatus stall_msg;

  // 填充基本信息
  stall_msg.header.stamp = current_time;
  stall_msg.header.frame_id = joint_.getName();
  stall_msg.is_stalled = is_stalled;
  stall_msg.joint_name = joint_.getName();

  // 填充当前状态
  stall_msg.current_velocity = std::abs(joint_.getVelocity());
  stall_msg.current_effort = std::abs(joint_.getEffort());

  // 填充阈值信息
  stall_msg.velocity_threshold = stalled_velocity_;
  stall_msg.effort_threshold = stalled_force_;

  // 填充持续时间信息
  if (stall_condition_active_ && !stall_condition_met_time_.isZero()) {
    stall_msg.stall_duration = (current_time - stall_condition_met_time_).toSec();
  } else {
    stall_msg.stall_duration = 0.0;
  }
  stall_msg.timeout_threshold = stall_timeout_;

  // 发布消息
  stall_status_pub_.publish(stall_msg);

  // 如果状态发生变化，记录日志
  static bool last_stall_state = false;
  if (is_stalled != last_stall_state) {
    if (is_stalled) {
      ROS_WARN("Gripper stall detected on joint '%s'! Velocity: %.4f (threshold: %.4f), Effort: %.4f (threshold: %.4f)",
               joint_.getName().c_str(), stall_msg.current_velocity, stalled_velocity_,
               stall_msg.current_effort, stalled_force_);
    } else {
      ROS_INFO("Gripper stall condition cleared on joint '%s'", joint_.getName().c_str());
    }
    last_stall_state = is_stalled;
  }
}

template <class HardwareInterface>
std::string GripperController<HardwareInterface>::stateToString(GripperState state)
{
  switch (state)
  {
  case GripperState::IDLE:    return "IDLE";
  case GripperState::MOVING:   return "MOVING";
  case GripperState::HOLDING:  return "HOLDING";
  case GripperState::ERROR:    return "ERROR";
  default:                     return "UNKNOWN";
  }
}

} // namespace gripper_controller
