//
// Created by lsy on 24-5-23.
//

#pragma once

#include <effort_controllers/joint_effort_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>

namespace temple_controller
{
template <typename CONTROL_CMD_MSGS>
class TempleController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
{
  enum ControllerState {
    NORMAL
  };
public:
  TempleController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void normal(const ros::Time& time, const ros::Duration& period);
  void commandCB(const CONTROL_CMD_MSGS& msg);

  int controllerState_ = NORMAL;
  ros::Time startTime_;
  bool stateChanged_ = false;
  ros::Subscriber cmdSub_;
  std::vector<hardware_interface::JointHandle> jointHandles_;
  realtime_tools::RealtimeBuffer<CONTROL_CMD_MSGS> cmdRtBuffer_{};
};

}  // namespace temple_controller