//
// Created by lsy on 24-5-23.
//

#pragma once

#include "gripper_controller/HardwareInterfaceAdapter.h"
#include <controller_interface/controller.h>
#include <effort_controllers/joint_effort_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>

#include "manipulator_msgs/GripperCmd.h"
#include "manipulator_msgs/GripperState.h"
#include "std_msgs/Float64.h"

namespace gripper_controller {

template <class HardwareInterface>
class gripperController : public controller_interface::Controller<HardwareInterface>
{
  enum ControllerState { NORMAL, SHUTDOWN };

public:
  gripperController();
  bool init(HardwareInterface *hw,ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void starting(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;

private:
  typedef HardwareInterfaceAdapter<HardwareInterface> HwIfaceAdapter;
  typedef realtime_tools::RealtimeBuffer<manipulator_msgs::GripperCmd> CmdBuff;

  HwIfaceAdapter hw_iface_adapter_;
  void normal(const ros::Time &time, const ros::Duration &period);
  void shutdown(const ros::Time &time, const ros::Duration &period);
  void commandCB(const manipulator_msgs::GripperCmd &msg);
  void enforceJointLimits(double &command);

  int loopCount_;

  int controllerState_ = NORMAL;
  std::unique_ptr<realtime_tools::RealtimePublisher<manipulator_msgs::GripperState>> statePub_;
  ros::Time startTime_;
  bool stateChanged_ = false;
  ros::Subscriber cmdSub_;

//   Use HardwareInterface::ResourceHandleType replace JointHandle
  typedef typename HardwareInterface::ResourceHandleType JointHandle;
  JointHandle jointHandle_;

  urdf::JointConstSharedPtr jointUrdf_;
  std::shared_ptr<CmdBuff>  cmdRtBuffer_{};
};

template <class HardwareInterface>
gripperController<HardwareInterface>::gripperController() {}

} // namespace gripper_controller

#include <gripper_controller/GripperControllerImpl.h>