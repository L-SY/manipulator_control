//
// Created by lsy on 24-5-23.
//

#include <string>
#include <pluginlib/class_list_macros.hpp>

#include "gripper_controller/GripperController.h"

namespace gripper_controller {
gripperController::gripperController() : loopCount_(0){}

bool gripperController::init(hardware_interface::RobotHW *robot_hw,
                             ros::NodeHandle &root_nh,
                             ros::NodeHandle &controller_nh) {
  auto *effortJointInterface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  std::string jointHandle_name;
  if (!controller_nh.getParam("joint", jointHandle_name)) {
    ROS_ERROR("No joint given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  if (!pidController_.init(ros::NodeHandle(controller_nh, "pid")))
    return false;

  statePub_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "state", 1));

  jointHandle_ = effortJointInterface->getHandle(jointHandle_name);

  cmdSub_ = controller_nh.subscribe("command", 1, &gripperController::commandCB,
                                    this);
  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", root_nh))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  jointUrdf_ = urdf.getJoint(jointHandle_name);
  if (!jointUrdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", jointHandle_name.c_str());
    return false;
  }

  std_msgs::Float64 initPos;
  initPos.data = jointHandle_.getPosition();
  cmdRtBuffer_.initRT(initPos);
  return true;
}

void gripperController::starting(const ros::Time & /*unused*/) {
  controllerState_ = ControllerState::NORMAL;
}

void gripperController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
  pidController_.setGains(p,i,d,i_max,i_min,antiwindup);
}

void gripperController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pidController_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void gripperController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool dummy;
  pidController_.getGains(p,i,d,i_max,i_min,dummy);
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
//    ROS_INFO_STREAM("normal Mode");
    double command_position = (cmdRtBuffer_.readFromRT())->data;

    double error;
    double commanded_effort;

    double current_position = jointHandle_.getPosition();

    enforceJointLimits(command_position);

    error = command_position - current_position;
    commanded_effort = pidController_.computeCommand(error, period);

    jointHandle_.setCommand(commanded_effort);

    if (loopCount_ % 10 == 0)
    {
      if(statePub_ && statePub_->trylock())
      {
        statePub_->msg_.data = jointHandle_.getPosition();
        statePub_->unlockAndPublish();
      }
    }
    loopCount_++;
  }

  void gripperController::enforceJointLimits(double &command)
  {
    // Check that this joint has applicable limits
    if (jointUrdf_->type == urdf::Joint::REVOLUTE || jointUrdf_->type == urdf::Joint::PRISMATIC)
    {
      if( command > jointUrdf_->limits->upper ) // above upper limnit
      {
        command = jointUrdf_->limits->upper;
      }
      else if( command < jointUrdf_->limits->lower ) // below lower limit
      {
        command = jointUrdf_->limits->lower;
      }
    }
  }

  void gripperController::commandCB(const std_msgs::Float64 &msg) {
    cmdRtBuffer_.writeFromNonRT(msg);
  }

} //namespace gripper_controller

PLUGINLIB_EXPORT_CLASS(gripper_controller::gripperController, controller_interface::ControllerBase)