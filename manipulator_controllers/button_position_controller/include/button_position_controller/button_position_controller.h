#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <realtime_tools/realtime_publisher.h>
#include "swingarm_hw/hardware_interface/button_panel_interface.h"

namespace button_position_controller
{

class ButtonPositionController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::ButtonPanelInterface>
{
public:
  ButtonPositionController();
  ~ButtonPositionController();

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh, ros::NodeHandle& controller_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

private:
  // Joint handle
  hardware_interface::JointHandle joint_;

  // Button panel handle
  hardware_interface::ButtonPanelHandle button_panel_;

  // Target positions
  double position_a_;
  double position_b_;
  double current_target_position_;

  // PID gains
  double p_gain_;
  double i_gain_;
  double d_gain_;

  // PID state
  double error_sum_;
  double last_error_;

  // Button state tracking
  bool last_button1_state_;
  bool last_button2_state_;

  // Status publishers
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> position_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> target_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>> button1_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>> button2_pub_;
};

} // namespace button_position_controller
