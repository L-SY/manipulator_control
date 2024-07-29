//
// Created by lsy on 24-5-23.
//

#include <string>
#include <pluginlib/class_list_macros.hpp>

#include "temple_controller/TempleController.h"

namespace temple_controller
{
template <typename CONTROL_CMD_MSGS>
bool TempleController<CONTROL_CMD_MSGS>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                              ros::NodeHandle& controller_nh)
{
  auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
  jointHandles_.push_back(effortJointInterface->getHandle("joint1"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint2"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint3"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint4"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint5"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint6"));

  cmdSub_ = controller_nh.subscribe("command", 1, &TempleController::commandCB, this);
  return true;
}

template <typename CONTROL_CMD_MSGS>
void TempleController<CONTROL_CMD_MSGS>::starting(const ros::Time& /*unused*/)
{
//  state_ = NORMAL;
}

template <typename CONTROL_CMD_MSGS>
void TempleController<CONTROL_CMD_MSGS>::update(const ros::Time& time, const ros::Duration& period)
{
  //  if (state_ != CONTROL_CMD_MSGS.mode)
  //  {
  //    state_ = CONTROL_CMD_MSGS.mode;
  //    stateChanged_ = true;
  //  }
//  switch (state_)
//  {
//    case temple_controller::NORMAL:
//      normal(time, period);
//      break;
//  }
}

template <typename CONTROL_CMD_MSGS>
void TempleController<CONTROL_CMD_MSGS>::normal(const ros::Time& time, const ros::Duration& period)
{
  ROS_INFO_STREAM("normal Mode");
}

template <typename CONTROL_CMD_MSGS>
void TempleController<CONTROL_CMD_MSGS>::commandCB(const CONTROL_CMD_MSGS& msg)
{
  cmdRtBuffer_.writeFromNonRT(msg);
}

}  // namespace temple_controller

PLUGINLIB_EXPORT_CLASS(temple_controller::TempleController<std_msgs::Float64>, controller_interface::ControllerBase)