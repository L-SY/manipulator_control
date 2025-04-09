#pragma once

#include <cassert>
#include <string>
#include <memory>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <manipulator_msgs/GripperStallStatus.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/multi_interface_controller.h>

#include <gripper_controller/hardware_interface_adapter.h>
#include <urdf/model.h>
#include <dynamic_reconfigure/server.h>
#include <gripper_controller/GripperControllerConfig.h>


namespace gripper_controller
{

/**
 * \brief 夹爪状态枚举
 */
enum class GripperState {
  IDLE,       // 空闲状态
  MOVING,     // 运动状态
  HOLDING,    // 夹持状态
  ERROR,       // 错误状态
  SELF_TEST   // 新增：自检状态
};

template <class HardwareInterface>
class GripperController : public controller_interface::MultiInterfaceController<HardwareInterface>
{
public:
  struct Commands
  {
    double position_;     // 目标位置
    double max_effort_;   // 最大允许力
  };

  GripperController();
  ~GripperController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  /**
     * \brief 获取当前夹爪状态
   */
  GripperState getState() const { return state_; }

  /**
     * \brief 将状态转换为字符串
   */
  static std::string stateToString(GripperState state);
  void triggerSelfTest();
private:
  void handleMovingState();
  void handleErrorState();
  void handleHoldingState();
  void handleIdleState();
  void handleSelfTestState(const ros::Time& time, const ros::Duration& period);
  void startSelfTest();

  ros::ServiceServer self_test_service_;
  bool triggerSelfTestCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // 自检相关变量
  bool self_test_active_ = false;
  int self_test_cycle_count_ = 0;
  int self_test_max_cycles_ = 3;
  double self_test_speed_factor_ = 1.0;
  ros::Time self_test_start_time_;
  ros::Time self_test_last_action_time_;
  enum class SelfTestPhase {
    OPENING,
    CLOSING,
    COMPLETED
  };
  SelfTestPhase self_test_phase_ = SelfTestPhase::OPENING;

  typedef HardwareInterfaceAdapter<HardwareInterface> HwIfaceAdapter;
  typedef typename HardwareInterface::ResourceHandleType HandleType;

  // 状态机相关
  GripperState state_;
  GripperState previous_state_;

  // 状态转换
  void transitionTo(GripperState new_state);

  // 状态检查
  void updateStallDetection(const ros::Time& current_time); // 更新堵转检测状态
  bool isStalled(const ros::Time& current_time); // 带时间参数的堵转检测

  bool isAtPosition(double target_position, double tolerance) const;
  bool isForceExceeded(double max_force) const;
  void publishState();
  // 命令回调
  void commandCB(const std_msgs::Float64::ConstPtr& msg);

  // 参数
  std::string urdf_string_;
  std::shared_ptr<urdf::Model> urdf_model_;

  // 修改参数名称
  double stalled_velocity_;
  double stalled_force_;
  double stall_timeout_;
  ros::Time stall_condition_met_time_; // 开始满足堵转条件的时间
  bool stall_condition_active_;  // 是否正在满足堵转条件
  bool is_stalled_;              // 堵转状态标志
  double last_position_error_;   // 上一次的位置误差
  double stall_position_threshold_; // 位置变化阈值，用于判断是否堵转
  bool isInstantStalled() const;
  void publishStallInformation(const ros::Time& current_time);

  // 限制参数（从URDF读取）
  double max_position_;
  double min_position_;
  double max_effort_;
  double max_velocity_;

  bool verbose_;                    // 调试标志
  std::string name_;               // 控制器名称

  double position_tolerance_;       // 位置容差

  // 运行时数据
  double target_position_;         // 目标位置
  double target_effort_;          // 目标力

  // 硬件接口
  HwIfaceAdapter hw_iface_adapter_;
  HandleType joint_;

  // 命令缓冲区
  Commands command_struct_;

  // ROS API
  ros::NodeHandle controller_nh_;
  ros::Publisher state_publisher_, stall_status_pub_;
  ros::Subscriber command_subscriber_;

  // 动态重构相关
  std::unique_ptr<dynamic_reconfigure::Server<gripper_controller::GripperControllerConfig>> dyn_reconfig_server_;
  void dynamicReconfigureCallback(gripper_controller::GripperControllerConfig& config, uint32_t level);

  // 添加URDF加载函数
  bool loadUrdf(ros::NodeHandle& root_nh);
};

} // namespace gripper_controller

// 包含实现文件
#include <gripper_controller/gripper_controller_impl.h>
