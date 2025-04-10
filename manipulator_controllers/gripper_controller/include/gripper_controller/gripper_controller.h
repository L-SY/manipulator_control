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
#include <manipulator_msgs/GripperStatus.h>
#include <manipulator_msgs/GripperCommand.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>

#include <gripper_controller/hardware_interface_adapter.h>
#include <urdf/model.h>
#include <dynamic_reconfigure/server.h>
#include <gripper_controller/GripperControllerConfig.h>

namespace gripper_controller
{
enum class GripperState
{
  IDLE,
  MOVING,
  HOLDING,
  ERROR,
  SELF_TEST,
};

enum class SelfTestPhase
{
  OPENING,
  CLOSING,
  COMPLETED
};

template <class HardwareInterface>
class GripperController : public controller_interface::MultiInterfaceController<HardwareInterface>
{
public:
  struct Commands
  {
    double position_;
    double max_effort_;
  };

  GripperController();
  ~GripperController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  void setCommand(double position, double effort);
  bool isAtPosition(double target_position) const;
  bool isForceExceeded(double max_force) const;

  void handleHoldingState();
  void handleIdleState();
  void handleMovingState();
  void handleErrorState();
  void triggerSelfTest();
  void startSelfTest();
  void handleSelfTestState(const ros::Time& time, const ros::Duration& period);

  void transitionTo(GripperState new_state);
  static std::string stateToString(GripperState state);

  bool isInstantStalled() const;
  void updateStallDetection(const ros::Time& current_time);
  bool isStalled(const ros::Time& current_time);

  void publishStatus(const ros::Time& current_time);

  void dynamicReconfigureCallback(gripper_controller::GripperControllerConfig& config, uint32_t level);
  void commandCB(const std_msgs::Float64::ConstPtr& msg);
  bool handleGripperCommandService(
      manipulator_msgs::GripperCommand::Request& req,
      manipulator_msgs::GripperCommand::Response& res);

  bool loadUrdf(ros::NodeHandle& root_nh);

private:
  GripperState state_;
  GripperState previous_state_;

  bool self_test_active_ = false;
  int self_test_cycle_count_ = 0;
  int self_test_max_cycles_ = 3;
  double self_test_speed_factor_ = 1.0;
  SelfTestPhase self_test_phase_ = SelfTestPhase::OPENING;

  typedef HardwareInterfaceAdapter<HardwareInterface> HwIfaceAdapter;
  typedef typename HardwareInterface::ResourceHandleType HandleType;
  HwIfaceAdapter hw_iface_adapter_;
  HandleType joint_;

  std::string urdf_string_;
  std::shared_ptr<urdf::Model> urdf_model_;
  double max_position_;
  double min_position_;
  double max_effort_;
  double max_velocity_;
  double position_tolerance_;
  double stalled_velocity_;
  double stalled_force_;
  double stall_timeout_;
  double release_offset_{0.01};
  ros::Time stall_condition_met_time_;
  bool stall_condition_active_;
  bool is_stalled_;
  double target_position_;
  double target_effort_;
  Commands command_struct_;

  ros::NodeHandle controller_nh_;
  ros::ServiceServer gripper_command_server_;
  ros::Subscriber command_subscriber_;
  std::shared_ptr<realtime_tools::RealtimePublisher<manipulator_msgs::GripperStatus>> status_pub_;
  std::unique_ptr<dynamic_reconfigure::Server<gripper_controller::GripperControllerConfig>> dyn_reconfig_server_;

  std::string name_;
  std::string current_command_;
  bool verbose_;
};

} // namespace gripper_controller

#include <gripper_controller/gripper_controller_impl.h>