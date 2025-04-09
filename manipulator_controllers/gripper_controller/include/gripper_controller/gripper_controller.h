#pragma once

// C++ standard
#include <cassert>
#include <stdexcept>
#include <string>
#include <memory>

// ROS
#include <ros/node_handle.h>
#include <ros/time.h>

// ROS messages
#include <control_msgs/GripperCommandAction.h>
#include <std_msgs/String.h>

// actionlib
#include <actionlib/server/action_server.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/multi_interface_controller.h>
// Project
#include <gripper_controller/hardware_interface_adapter.h>
#include <manipulator_msgs/GripperCommandAction.h>
#include <manipulator_msgs/GripperCommandActionFeedback.h>
#include <manipulator_msgs/GripperCommandActionFeedback.h>

namespace gripper_controller
{

/**
 * \brief 夹爪状态枚举
 */
enum class GripperState {
  OPENED,             // 1. 打开的
  OPENING,            // 2. 正在打开的
  FULLY_CLOSED,       // 3. 完全闭合
  GRIPPING,           // 4. 正在夹取的
  HOLDING,            // 5. 夹住的
  RELEASED,           // 6. 松开
  POSITIONED,         // 7. 指定位置
  SELF_TESTING,       // 8. 夹爪自检
  SELF_TEST_FAILED,   // 9. 自检失败
  STALLED             // 10. 异常堵转
};

/**
 * \brief 夹爪命令枚举
 */
enum class GripperCommand {
  OPEN,               // 打开命令
  GRIP,               // 夹取命令
  RELEASE,            // 松开命令
  POSITION,           // 指定位置命令
  SELF_TEST           // 自检命令
};

/**
 * \brief 增强型夹爪控制器，支持完整的状态机和多种命令模式
 *
 * \tparam HardwareInterface 控制器硬件接口。目前支持 hardware_interface::PositionJointInterface 和
 * hardware_interface::EffortJointInterface。
 */
template <class HardwareInterface>
class GripperController : public controller_interface::MultiInterfaceController<HardwareInterface>
{
public:
  /**
   * \brief 存储位置和最大力的结构体，便于实时缓冲区使用
   */
  struct Commands
  {
    double position_;    // 目标位置
    double max_effort_;  // 最大允许力
    double speed_;       // 运动速度
  };

  GripperController();
  ~GripperController() = default;

  /** \name 非实时安全函数
   *\{*/
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /*\}*/

  /** \name 实时安全函数
   *\{*/
  /** \brief 保持当前位置 */
  void starting(const ros::Time& time) override;

  /** \brief 取消活动的动作目标 */
  void stopping(const ros::Time& time) override;

  /** \brief 更新控制器 */
  void update(const ros::Time& time, const ros::Duration& period) override;
  /*\}*/

  /**
   * \brief 执行夹爪命令
   * \param cmd 命令类型
   * \param position 目标位置
   * \param max_effort 最大力
   * \param speed 运动速度
   * \return 是否成功接收命令
   */
  bool executeCommand(GripperCommand cmd, double position = 0.0, double max_effort = 0.0, double speed = 0.0);

  /**
   * \brief 获取当前夹爪状态
   * \return 当前状态
   */
  GripperState getState() const { return state_; }

  /**
   * \brief 将状态转换为字符串
   * \param state 夹爪状态
   * \return 状态字符串
   */
  static std::string stateToString(GripperState state);

private:
  typedef actionlib::ActionServer<manipulator_msgs::GripperCommandAction> ActionServer;
  typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
  typedef ActionServer::GoalHandle GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<manipulator_msgs::GripperCommandAction>        RealtimeGoalHandle;
  typedef boost::shared_ptr<realtime_tools::RealtimeServerGoalHandle<manipulator_msgs::GripperCommandAction>> RealtimeGoalHandlePtr;

  typedef HardwareInterfaceAdapter<HardwareInterface> HwIfaceAdapter;

  // 状态机相关
  GripperState state_;
  GripperState previous_state_;

  // 命令处理
  void handleOpenCommand(double speed);
  void handleGripCommand(double max_effort, double speed);
  void handleReleaseCommand();
  void handlePositionCommand(double position, double max_effort);
  void handleSelfTestCommand();

  // 状态转换
  void transitionTo(GripperState new_state);

  // 状态检查
  bool isStalled(const ros::Time& time);
  bool isAtPosition(double target_position, double tolerance);
  bool isForceExceeded(double max_force);

  // 自检相关
  void updateSelfTest(const ros::Time& time, const ros::Duration& period);
  int self_test_cycle_count_;
  int self_test_max_cycles_;
  bool self_test_opening_phase_;

  // 命令更新
  void updateCommand(const ros::Time& time, const ros::Duration& period);

  // 参数
  bool verbose_;                  // 调试标志
  std::string name_;              // 控制器名称
  double max_position_;           // 夹爪最大开度 (从URDF)
  double min_position_;           // 夹爪最小开度 (从URDF)
  double default_speed_;          // 默认运动速度
  double position_tolerance_;     // 位置容差
  double force_threshold_;        // 力阈值
  double stall_velocity_threshold_; // 堵转速度阈值
  double stall_timeout_;          // 堵转超时
  double default_max_effort_;     // 默认最大力
  double release_offset_;         // 松开时的偏移量

  // 运行时数据
  double target_position_;        // 目标位置
  double target_effort_;          // 目标力
  double computed_command_;       // 计算出的命令
  ros::Time last_movement_time_;  // 上次运动时间

  // 硬件接口
  HwIfaceAdapter hw_iface_adapter_;

  // 命令缓冲区
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_, command_struct_rt_;

  // ROS API
  ros::NodeHandle controller_nh_;
  ActionServerPtr action_server_;
  RealtimeGoalHandlePtr rt_active_goal_;
  boost::shared_ptr<manipulator_msgs::GripperCommandResult> pre_alloc_result_;

  ros::Timer goal_handle_timer_;
  ros::Publisher state_publisher_;

  // 回调函数
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void preemptActiveGoal();
  void setHoldPosition(const ros::Time& time);

  // 反馈和结果
  void publishFeedback(const ros::Time& time);
  void publishState();
  void completeGoal(bool success, const std::string& message);
  void checkForSuccess(const ros::Time& time, double error_position, double current_position, double current_velocity);
  typedef typename HardwareInterface::ResourceHandleType HandleType;
  HandleType joint_;
};

} // namespace gripper_controller

// 包含实现文件
#include <gripper_controller/gripper_controller_impl.h>
