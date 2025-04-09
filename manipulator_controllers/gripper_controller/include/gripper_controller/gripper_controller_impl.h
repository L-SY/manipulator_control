#pragma once

namespace gripper_controller
{

template <class HardwareInterface>
GripperController<HardwareInterface>::GripperController()
    : state_(GripperState::IDLE)
      , previous_state_(GripperState::IDLE)
      , verbose_(false)
      , max_position_(0.0)
      , min_position_(0.0)
      , position_tolerance_(0.01)
      , force_threshold_(10.0)
      , default_max_effort_(10.0)
      , target_position_(0.0)
      , target_effort_(0.0)
{
  command_struct_.position_ = 0.0;
  command_struct_.max_effort_ = 0.0;
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::init(hardware_interface::RobotHW* robot_hw,
                                                ros::NodeHandle& /*root_nh*/, ros::NodeHandle& controller_nh)
{
  controller_nh_ = controller_nh;
  name_ = controller_nh_.getNamespace();

  // 获取关节名称
  std::string joint_name;
  if (!controller_nh_.getParam("joint", joint_name)) {
    ROS_ERROR_STREAM_NAMED(name_, "No joint given in namespace: " << controller_nh_.getNamespace());
    return false;
  }

  // 加载控制器参数
  controller_nh_.param("verbose", verbose_, verbose_);
  controller_nh_.param("max_position", max_position_, max_position_);
  controller_nh_.param("min_position", min_position_, min_position_);
  controller_nh_.param("position_tolerance", position_tolerance_, position_tolerance_);
  controller_nh_.param("force_threshold", force_threshold_, force_threshold_);
  controller_nh_.param("default_max_effort", default_max_effort_, default_max_effort_);

  if (max_position_ <= min_position_) {
    ROS_ERROR_STREAM_NAMED(name_, "Invalid position limits");
    return false;
  }

  // 初始化硬件接口
  auto* hw = robot_hw->get<HardwareInterface>();
  joint_ = hw->getHandle(joint_name);
  hw_iface_adapter_.init(joint_, controller_nh_);

  // 初始化命令
  command_struct_.position_ = joint_.getPosition();
  command_struct_.max_effort_ = default_max_effort_;

  // 初始化ROS接口
  command_subscriber_ = controller_nh_.subscribe<std_msgs::Float64>(
      "command", 1, &GripperController::commandCB, this);
  state_publisher_ = controller_nh_.advertise<std_msgs::String>("state", 1);

  return true;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::starting(const ros::Time& /*time*/)
{
  // 初始化状态
  state_ = GripperState::IDLE;
  previous_state_ = GripperState::IDLE;

  // 使用当前位置
  target_position_ = joint_.getPosition();
  target_effort_ = default_max_effort_;

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
void GripperController<HardwareInterface>::update(const ros::Time& /*time*/, const ros::Duration& period)
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

  hw_iface_adapter_.updateCommand(period, target_position_, target_effort_);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleMovingState()
{
  if (isAtPosition(target_position_, position_tolerance_)) {
    transitionTo(GripperState::HOLDING);
    return;
  }

  if (isStalled()) {
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
  target_effort_ = default_max_effort_;

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
bool GripperController<HardwareInterface>::isStalled() const
{
  // 简单的堵转检测：当前速度接近零且施加的力超过阈值
  return (std::abs(joint_.getVelocity()) < 0.001 &&
          std::abs(joint_.getEffort()) > force_threshold_);
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
