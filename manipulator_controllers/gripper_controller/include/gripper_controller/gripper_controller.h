#pragma once

#include <cassert>
#include <string>
#include <memory>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/multi_interface_controller.h>

#include <gripper_controller/hardware_interface_adapter.h>

namespace gripper_controller
{

/**
 * \brief 夹爪状态枚举
 */
enum class GripperState {
  IDLE,       // 空闲状态
  MOVING,     // 运动状态
  HOLDING,    // 夹持状态
  ERROR       // 错误状态
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

private:
  typedef HardwareInterfaceAdapter<HardwareInterface> HwIfaceAdapter;
  typedef typename HardwareInterface::ResourceHandleType HandleType;

  // 状态机相关
  GripperState state_;
  GripperState previous_state_;

  // 状态转换
  void transitionTo(GripperState new_state);

  // 状态检查
  bool isStalled() const;
  bool isAtPosition(double target_position, double tolerance) const;
  bool isForceExceeded(double max_force) const;
  void publishState();
  // 命令回调
  void commandCB(const std_msgs::Float64::ConstPtr& msg);

  // 参数
  bool verbose_;                    // 调试标志
  std::string name_;               // 控制器名称
  double max_position_;            // 夹爪最大开度
  double min_position_;            // 夹爪最小开度
  double position_tolerance_;       // 位置容差
  double force_threshold_;         // 力阈值
  double default_max_effort_;      // 默认最大力

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
  ros::Publisher state_publisher_;
  ros::Subscriber command_subscriber_;
};

} // namespace gripper_controller

// 包含实现文件
#include <gripper_controller/gripper_controller_impl.h>
