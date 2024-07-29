//
// Created by lsy on 24-5-23.
//

#include <string>
#include <pluginlib/class_list_macros.hpp>

#include "gripper_controller/GripperController.h"

namespace gripper_controller {
bool gripperController::init(hardware_interface::RobotHW *robot_hw,
                             ros::NodeHandle &root_nh,
                             ros::NodeHandle &controller_nh) {
  auto *effortJointInterface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  std::string joint_name;
  if (!controller_nh.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  jointHandle_ = effortJointInterface->getHandle(joint_name);

  cmdSub_ = controller_nh.subscribe("command", 1, &gripperController::commandCB,
                                    this);
  return true;
}

void gripperController::starting(const ros::Time & /*unused*/) {
  controllerState_ = ControllerState::NORMAL;
}

void gripperController::update(const ros::Time &time,
                               const ros::Duration &period) {
  //  if (controllerState_ != CONTROL_CMD_MSGS.mode)
  //  {
  //    controllerState_ = CONTROL_CMD_MSGS.mode;
  //    stateChanged_ = true;
  //  }
  switch (controllerState_) {
  case ControllerState::NORMAL:
    normal(time, period);
    break;
    //  }
  }
}

  void gripperController::normal(const ros::Time &time,
                                 const ros::Duration &period) {
    ROS_INFO_STREAM("normal Mode");
  }

  void gripperController::commandCB(const std_msgs::Float64 &msg) {
    cmdRtBuffer_.writeFromNonRT(msg);
  }

} //namespace gripper_controller

PLUGINLIB_EXPORT_CLASS(gripper_controller::gripperController, controller_interface::ControllerBase)