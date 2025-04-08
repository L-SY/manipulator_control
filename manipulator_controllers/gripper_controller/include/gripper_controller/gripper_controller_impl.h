#pragma once

namespace gripper_controller
{

template <class HardwareInterface>
GripperController<HardwareInterface>::GripperController()
    : state_(GripperState::OPENED)
      , previous_state_(GripperState::OPENED)
      , self_test_cycle_count_(0)
      , self_test_max_cycles_(3)
      , self_test_opening_phase_(true)
      , verbose_(false)
      , max_position_(0.0)
      , min_position_(0.0)
      , default_speed_(0.1)
      , position_tolerance_(0.01)
      , force_threshold_(10.0)
      , stall_velocity_threshold_(0.001)
      , stall_timeout_(0.2)
      , default_max_effort_(10.0)
      , release_offset_(0.02)
      , target_position_(0.0)
      , target_effort_(0.0)
      , computed_command_(0.0)
{
  command_struct_.position_ = 0.0;
  command_struct_.max_effort_ = 0.0;
  command_struct_.speed_ = 0.0;
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& /*root_nh*/, ros::NodeHandle& controller_nh)
{
  // 存储控制器节点句柄
  controller_nh_ = controller_nh;
  name_ = controller_nh_.getNamespace();

  // 获取关节名称
  std::string joint_name;
  if (!controller_nh_.getParam("joint", joint_name))
  {
    ROS_ERROR_STREAM_NAMED(name_, "No joint given in namespace: " << controller_nh_.getNamespace());
    return false;
  }

  // 加载控制器参数
  controller_nh_.param("verbose", verbose_, verbose_);

  // 加载位置限制
  controller_nh_.param("max_position", max_position_, max_position_);
  controller_nh_.param("min_position", min_position_, min_position_);

  if (max_position_ <= min_position_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Invalid position limits: min_position (" << min_position_
                                                                            << ") must be less than max_position (" << max_position_ << ")");
    return false;
  }

  // 加载其他参数
  controller_nh_.param("default_speed", default_speed_, default_speed_);
  controller_nh_.param("position_tolerance", position_tolerance_, position_tolerance_);
  controller_nh_.param("force_threshold", force_threshold_, force_threshold_);
  controller_nh_.param("stall_velocity_threshold", stall_velocity_threshold_, stall_velocity_threshold_);
  controller_nh_.param("stall_timeout", stall_timeout_, stall_timeout_);
  controller_nh_.param("default_max_effort", default_max_effort_, default_max_effort_);
  controller_nh_.param("release_offset", release_offset_, release_offset_);

  hw_iface_adapter_.init(robot_hw->get<HardwareInterface>(), controller_nh_);

  command_struct_.position_ = hw_iface_adapter_.getPosition();
  command_struct_.max_effort_ = default_max_effort_;
  command_struct_.speed_ = default_speed_;
  command_.initRT(command_struct_);

  // 初始化预分配的结果
  pre_alloc_result_.reset(new manipulator_msgs::GripperCommandResult());

  // 启动Action服务器
  action_server_.reset(new ActionServer(controller_nh_, "gripper_cmd",
                                        boost::bind(&GripperController::goalCB, this, _1),
                                        boost::bind(&GripperController::cancelCB, this, _1),
                                        false));
  action_server_->start();

  // 状态发布器
  state_publisher_ = controller_nh_.advertise<std_msgs::String>("state", 1);

  // 设置初始状态
  state_ = GripperState::OPENED;
  publishState();

  return true;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::starting(const ros::Time& time)
{
  // 更新当前命令
  setHoldPosition(time);

  // 重置状态
  state_ = GripperState::OPENED;
  previous_state_ = GripperState::OPENED;

  // 重置运行时变量
  last_movement_time_ = time;

  if (verbose_)
    ROS_INFO_STREAM_NAMED(name_, "Starting controller");
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::stopping(const ros::Time& /*time*/)
{
  // 取消活动目标
  preemptActiveGoal();

  if (verbose_)
    ROS_INFO_STREAM_NAMED(name_, "Stopping controller");
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::update(const ros::Time& time, const ros::Duration& period)
{
  // 获取当前命令
  command_struct_rt_ = *(command_.readFromRT());

  // 获取当前状态
  double current_position = hw_iface_adapter_.getPosition();
  double current_velocity = hw_iface_adapter_.getVelocity();
  double current_effort = hw_iface_adapter_.getEffort();

  // 状态机逻辑
  switch (state_)
  {
  case GripperState::OPENING:
    if (isStalled(time)) {
      transitionTo(GripperState::STALLED);
    }
    else if (isAtPosition(max_position_, position_tolerance_)) {
      transitionTo(GripperState::OPENED);
    }
    break;

  case GripperState::GRIPPING:
    if (isStalled(time)) {
      if (current_position <= min_position_ + position_tolerance_) {
        transitionTo(GripperState::FULLY_CLOSED);
      } else if (isForceExceeded(target_effort_)) {
        transitionTo(GripperState::HOLDING);
      } else {
        transitionTo(GripperState::STALLED);
      }
    }
    else if (current_position <= min_position_ + position_tolerance_) {
      transitionTo(GripperState::FULLY_CLOSED);
    }
    else if (isForceExceeded(target_effort_)) {
      transitionTo(GripperState::HOLDING);
    }
    break;

  case GripperState::SELF_TESTING:
    if (isStalled(time)) {
      transitionTo(GripperState::SELF_TEST_FAILED);
    }
    else {
      updateSelfTest(time, period);
    }
    break;

  case GripperState::POSITIONED:
    // 检查是否到达目标位置
    if (isAtPosition(target_position_, position_tolerance_)) {
      // 如果方向是往内夹并且超过力阈值，认为夹住了物体
      if (target_position_ < current_position && isForceExceeded(target_effort_)) {
        transitionTo(GripperState::HOLDING);
      }
    }
    else if (isStalled(time)) {
      if (isForceExceeded(target_effort_)) {
        transitionTo(GripperState::HOLDING);
      } else {
        transitionTo(GripperState::STALLED);
      }
    }
    break;

  case GripperState::RELEASED:
    // 检查是否到达释放位置
    if (isAtPosition(target_position_, position_tolerance_)) {
      transitionTo(GripperState::OPENED);
    }
    else if (isStalled(time)) {
      transitionTo(GripperState::STALLED);
    }
    break;

  default:
    // 其他状态不需要特殊处理
    break;
  }

  // 更新命令
  updateCommand(time, period);

  // 发布反馈
  publishFeedback(time);

  // 检查目标是否完成
  if (rt_active_goal_) {
    double error_position = std::abs(target_position_ - current_position);
    checkForSuccess(time, error_position, current_position, current_velocity);
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::updateCommand(const ros::Time& /*time*/, const ros::Duration& period)
{
  // 根据当前状态计算命令
  switch (state_)
  {
  case GripperState::OPENING:
    computed_command_ = max_position_;
    break;

  case GripperState::GRIPPING:
    computed_command_ = min_position_;
    break;

  case GripperState::SELF_TESTING:
    // 命令已在updateSelfTest中设置
    computed_command_ = target_position_;
    break;

  case GripperState::POSITIONED:
  case GripperState::RELEASED:
    computed_command_ = target_position_;
    break;

  case GripperState::OPENED:
  case GripperState::FULLY_CLOSED:
  case GripperState::HOLDING:
  case GripperState::STALLED:
  case GripperState::SELF_TEST_FAILED:
    // 保持当前位置
    computed_command_ = hw_iface_adapter_.getPosition();
    break;
  }

  // 将命令发送到硬件接口适配器
  hw_iface_adapter_.updateCommand(period, computed_command_, target_effort_);

  // 更新最后移动时间
  double velocity = hw_iface_adapter_.getVelocity();
  if (std::abs(velocity) > stall_velocity_threshold_) {
    last_movement_time_ = ros::Time::now();
  }
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::executeCommand(
    GripperCommand cmd, double position, double max_effort, double speed)
{
  switch (cmd)
  {
  case GripperCommand::OPEN:
    handleOpenCommand(speed);
    return true;

  case GripperCommand::GRIP:
    handleGripCommand(max_effort, speed);
    return true;

  case GripperCommand::RELEASE:
    handleReleaseCommand();
    return true;

  case GripperCommand::POSITION:
    handlePositionCommand(position, max_effort);
    return true;

  case GripperCommand::SELF_TEST:
    handleSelfTestCommand();
    return true;

  default:
    ROS_ERROR_STREAM_NAMED(name_, "Unknown gripper command");
    return false;
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleOpenCommand(double speed)
{
  if (speed <= 0.0) speed = default_speed_;

  target_position_ = max_position_;
  target_effort_ = default_max_effort_;

  // 更新命令结构
  command_struct_.position_ = target_position_;
  command_struct_.max_effort_ = target_effort_;
  command_struct_.speed_ = speed;
  command_.writeFromNonRT(command_struct_);

  transitionTo(GripperState::OPENING);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleGripCommand(double max_effort, double speed)
{
  if (state_ != GripperState::OPENED) {
    // 先执行打开
    handleOpenCommand(speed);
    ros::Duration(0.5).sleep(); // 等待一小段时间开始打开
  }

  target_position_ = min_position_;
  target_effort_ = (max_effort > 0.0) ? max_effort : default_max_effort_;

  // 更新命令结构
  command_struct_.position_ = target_position_;
  command_struct_.max_effort_ = target_effort_;
  command_struct_.speed_ = (speed > 0.0) ? speed : default_speed_;
  command_.writeFromNonRT(command_struct_);

  transitionTo(GripperState::GRIPPING);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleReleaseCommand()
{
  if (state_ == GripperState::HOLDING) {
    // 微微张开，让物体掉落
    double current_position = hw_iface_adapter_.getPosition();
    target_position_ = current_position + release_offset_;
    if (target_position_ > max_position_)
      target_position_ = max_position_;

    // 更新命令结构
    command_struct_.position_ = target_position_;
    command_struct_.max_effort_ = default_max_effort_;
    command_struct_.speed_ = default_speed_;
    command_.writeFromNonRT(command_struct_);

    transitionTo(GripperState::RELEASED);
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handlePositionCommand(double position, double max_effort)
{
  // 检查位置是否在范围内
  if (position < min_position_ || position > max_position_) {
    ROS_WARN_STREAM_NAMED(name_, "Position command out of range [" << min_position_ << ", "
                                                                   << max_position_ << "]: " << position);
    return;
  }

  target_position_ = position;
  target_effort_ = (max_effort > 0.0) ? max_effort : default_max_effort_;

  // 更新命令结构
  command_struct_.position_ = target_position_;
  command_struct_.max_effort_ = target_effort_;
  command_struct_.speed_ = default_speed_;
  command_.writeFromNonRT(command_struct_);

  transitionTo(GripperState::POSITIONED);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::handleSelfTestCommand()
{
  self_test_cycle_count_ = 0;
  self_test_max_cycles_ = 3;  // 循环三次
  self_test_opening_phase_ = true;  // 从打开阶段开始

  // 设置初始目标
  target_position_ = max_position_;
  target_effort_ = default_max_effort_;

  // 更新命令结构
  command_struct_.position_ = target_position_;
  command_struct_.max_effort_ = target_effort_;
  command_struct_.speed_ = default_speed_;
  command_.writeFromNonRT(command_struct_);

  transitionTo(GripperState::SELF_TESTING);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::updateSelfTest(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  if (self_test_opening_phase_) {
    // 打开阶段
    target_position_ = max_position_;

    if (isAtPosition(max_position_, position_tolerance_)) {
      self_test_opening_phase_ = false;  // 切换到关闭阶段
    }
  } else {
    // 关闭阶段
    target_position_ = min_position_;

    if (isAtPosition(min_position_, position_tolerance_)) {
      self_test_opening_phase_ = true;  // 切换回打开阶段
      self_test_cycle_count_++;  // 完成一个循环

      if (self_test_cycle_count_ >= self_test_max_cycles_) {
        // 自检完成
        transitionTo(GripperState::OPENED);
      }
    }
  }

  // 更新命令结构
  command_struct_.position_ = target_position_;
  command_.writeFromNonRT(command_struct_);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::transitionTo(GripperState new_state)
{
  if (state_ == new_state) return;

  previous_state_ = state_;
  state_ = new_state;

  if (verbose_)
    ROS_INFO_STREAM_NAMED(name_, "Gripper state transition: " << stateToString(previous_state_)
                                                              << " -> " << stateToString(state_));

  // 发布状态更新
  publishState();

  // 状态转换后的特殊处理
  switch (new_state)
  {
  case GripperState::OPENED:
    if (rt_active_goal_ && previous_state_ == GripperState::OPENING) {
      completeGoal(true, "Gripper fully opened");
    }
    break;

  case GripperState::FULLY_CLOSED:
    if (rt_active_goal_) {
      completeGoal(true, "Gripper fully closed");
    }
    break;

  case GripperState::HOLDING:
    if (rt_active_goal_) {
      completeGoal(true, "Object grasped successfully");
    }
    break;

  case GripperState::STALLED:
    if (rt_active_goal_) {
      completeGoal(false, "Gripper stalled");
    }
    break;

  case GripperState::SELF_TEST_FAILED:
    if (rt_active_goal_) {
      completeGoal(false, "Self-test failed");
    }
    break;

  default:
    break;
  }
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::isStalled(const ros::Time& time)
{
  // 检查自上次运动以来的时间
  double time_since_last_movement = (time - last_movement_time_).toSec();

  // 如果速度低于阈值并且超过超时时间，认为堵转
  return (std::abs(hw_iface_adapter_.getVelocity()) < stall_velocity_threshold_ &&
          time_since_last_movement > stall_timeout_);
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::isAtPosition(double target_position, double tolerance)
{
  double current_position = hw_iface_adapter_.getPosition();
  return std::abs(current_position - target_position) < tolerance;
}

template <class HardwareInterface>
bool GripperController<HardwareInterface>::isForceExceeded(double max_force)
{
  double current_effort = std::abs(hw_iface_adapter_.getEffort());
  return current_effort >= max_force;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::goalCB(GoalHandle gh)
{
  if (!this->isRunning()) {
    ROS_ERROR_STREAM_NAMED(name_, "Can't accept new action goals. Controller is not running.");
    manipulator_msgs::GripperCommandResult result;
    result.success = false;
    result.message = "Controller is not running";
    gh.setRejected(result);
    return;
  }

  // 接受新目标
  preemptActiveGoal();
  gh.setAccepted();

  // 解析命令
  const auto& goal = gh.getGoal();

  if (goal->position >= max_position_ - position_tolerance_) {
    // 位置接近最大值，视为打开命令
    executeCommand(GripperCommand::OPEN);
  }
  else if (goal->position <= min_position_ + position_tolerance_) {
    // 位置接近最小值，视为夹取命令
    executeCommand(GripperCommand::GRIP, 0.0, goal->max_effort);
  }
  else {
    // 其他位置，视为定位命令
    executeCommand(GripperCommand::POSITION, goal->position, goal->max_effort);
  }

  // 设置活动目标
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  rt_active_goal_ = rt_goal;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_goal = rt_active_goal_;

  // 检查当前目标是否存在且与取消的目标匹配
  if (current_goal && current_goal->gh_ == gh) {
    // 取消当前目标
    preemptActiveGoal();
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_goal = rt_active_goal_;

  if (current_goal) {
    // 设置已抢占
    current_goal->gh_.setCanceled();

    // 重置活动目标
    rt_active_goal_.reset();
  }
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::setHoldPosition(const ros::Time& /*time*/)
{
  // 获取当前位置
  double current_position = hw_iface_adapter_.getPosition();

  // 更新命令
  command_struct_.position_ = current_position;
  command_struct_.max_effort_ = default_max_effort_;
  command_struct_.speed_ = default_speed_;
  command_.writeFromNonRT(command_struct_);

  // 更新目标位置
  target_position_ = current_position;
  target_effort_ = default_max_effort_;
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::publishFeedback(const ros::Time& time)
{
  if (!rt_active_goal_) return;

  // 获取当前状态
  double current_position = hw_iface_adapter_.getPosition();
  double current_effort = hw_iface_adapter_.getEffort();

  // 创建并填充反馈
  manipulator_msgs::GripperCommandFeedback feedback;
  feedback.position = current_position;
  feedback.effort = current_effort;

  // 注意：您的消息中 stalled 和 reached_goal 是 float64 类型，而不是 bool
  feedback.stalled = (state_ == GripperState::STALLED) ? 1.0 : 0.0;
  feedback.reached_goal = (state_ == GripperState::OPENED ||
                           state_ == GripperState::FULLY_CLOSED ||
                           state_ == GripperState::HOLDING) ? 1.0 : 0.0;

  // 创建共享指针
  manipulator_msgs::GripperCommandFeedbackConstPtr feedback_ptr(
      new manipulator_msgs::GripperCommandFeedback(feedback));

  // 发布反馈
  rt_active_goal_->setFeedback(feedback_ptr);
}



template <class HardwareInterface>
void GripperController<HardwareInterface>::publishState()
{
  std_msgs::String state_msg;
  state_msg.data = stateToString(state_);
  state_publisher_.publish(state_msg);
}

template <class HardwareInterface>
void GripperController<HardwareInterface>::completeGoal(bool success, const std::string& message)
{
  if (!rt_active_goal_) return;

  // 获取当前状态
  double current_position = hw_iface_adapter_.getPosition();
  double current_effort = hw_iface_adapter_.getEffort();

  // 填充结果
  manipulator_msgs::GripperCommandResult result;
  result.position = current_position;
  result.effort = current_effort;
  result.success = success;
  result.message = message;

  if (success) {
    rt_active_goal_->gh_.setSucceeded(result);
  } else {
    rt_active_goal_->gh_.setAborted(result);
  }

  // 重置活动目标
  rt_active_goal_.reset();

  if (verbose_)
    ROS_INFO_STREAM_NAMED(name_, "Goal completed: " << message);
}


template <class HardwareInterface>
void GripperController<HardwareInterface>::checkForSuccess(const ros::Time& time, double error_position,
                                                           double current_position, double current_velocity)
{
  // 只在某些状态下检查成功
  if (state_ != GripperState::OPENING &&
      state_ != GripperState::GRIPPING &&
      state_ != GripperState::POSITIONED) {
    return;
  }

  // 检查是否达到目标位置
  if (error_position < position_tolerance_) {
    // 位置达到，根据状态设置结果
    switch (state_) {
    case GripperState::OPENING:
      transitionTo(GripperState::OPENED);
      break;

    case GripperState::GRIPPING:
      if (current_position <= min_position_ + position_tolerance_) {
        transitionTo(GripperState::FULLY_CLOSED);
      } else {
        transitionTo(GripperState::HOLDING);
      }
      break;

    case GripperState::POSITIONED:
      // 如果是向内夹并且力超过阈值，认为夹住了物体
      if (target_position_ < current_position && isForceExceeded(target_effort_)) {
        transitionTo(GripperState::HOLDING);
      }
      break;

    default:
      break;
    }
  }
  // 检查是否堵转
  else if (isStalled(time)) {
    if (state_ == GripperState::GRIPPING && isForceExceeded(target_effort_)) {
      // 在夹取过程中力超过阈值，认为夹住了物体
      transitionTo(GripperState::HOLDING);
    } else {
      // 其他情况认为堵转
      transitionTo(GripperState::STALLED);
    }
  }
}

template <class HardwareInterface>
std::string GripperController<HardwareInterface>::stateToString(GripperState state)
{
  switch (state)
  {
  case GripperState::OPENED:          return "OPENED";
  case GripperState::OPENING:         return "OPENING";
  case GripperState::FULLY_CLOSED:    return "FULLY_CLOSED";
  case GripperState::GRIPPING:        return "GRIPPING";
  case GripperState::HOLDING:         return "HOLDING";
  case GripperState::RELEASED:        return "RELEASED";
  case GripperState::POSITIONED:      return "POSITIONED";
  case GripperState::SELF_TESTING:    return "SELF_TESTING";
  case GripperState::SELF_TEST_FAILED:return "SELF_TEST_FAILED";
  case GripperState::STALLED:         return "STALLED";
  default:                            return "UNKNOWN";
  }
}

} // namespace gripper_controller
