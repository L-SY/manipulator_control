//
// Created by lsy on 24-8-28.
//

#pragma once

// URDF
#include <urdf/model.h>

namespace gripper_controller {
namespace internal {
std::string getLeafNamespace(const ros::NodeHandle &nh) {
  const std::string complete_ns = nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

urdf::ModelSharedPtr getUrdf(const ros::NodeHandle &nh,
                             const std::string &param_name) {
  urdf::ModelSharedPtr urdf(new urdf::Model);

  std::string urdf_str;
  // Check for robot_description in proper namespace
  if (nh.getParam(param_name, urdf_str)) {
    if (!urdf->initString(urdf_str)) {
      ROS_ERROR_STREAM("Failed to parse URDF contained in '"
                       << param_name << "' parameter (namespace: "
                       << nh.getNamespace() << ").");
      return urdf::ModelSharedPtr();
    }
  }
  // Check for robot_description in root
  else if (!urdf->initParam("robot_description")) {
    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name
                                                           << "' parameter");
    return urdf::ModelSharedPtr();
  }
  return urdf;
}

std::vector<urdf::JointConstSharedPtr>
getUrdfJoints(const urdf::Model &urdf,
              const std::vector<std::string> &joint_names) {
  std::vector<urdf::JointConstSharedPtr> out;
  for (const auto &joint_name : joint_names) {
    urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joint_name);
    if (urdf_joint) {
      out.push_back(urdf_joint);
    } else {
      ROS_ERROR_STREAM("Could not find joint '" << joint_name
                                                << "' in URDF model.");
      return std::vector<urdf::JointConstSharedPtr>();
    }
  }
  return out;
}
} // namespace internal

template <class HardwareInterface>
inline void
gripperController<HardwareInterface>::starting(const ros::Time &time) {
  manipulator_msgs::GripperCmd initCmd;
  initCmd.stamp = ros::Time::now();
  initCmd.mode = NORMAL;
  initCmd.des_pos = 0.;
  initCmd.des_vel = 0.;
  initCmd.des_eff = 0.;
  cmdRtBuffer_->initRT(initCmd);

  // Hardware interface adapter
  hw_iface_adapter_.starting(ros::Time(0.0));
}

template <class HardwareInterface>
bool gripperController<HardwareInterface>::init(
    HardwareInterface *hw, ros::NodeHandle &root_nh,
    ros::NodeHandle &controller_nh) {
  using namespace internal;
  std::string name = getLeafNamespace(controller_nh);

  // URDF joints
  urdf::ModelSharedPtr urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) {
    return false;
  }

  // Initialize members
  // Joint handle
  std::string jointHandle_name;
  if (!controller_nh.getParam("joint", jointHandle_name)) {
    ROS_ERROR("No joint given (namespace: %s)",
              controller_nh.getNamespace().c_str());
    return false;
  }
  try {
    jointHandle_ = hw->getHandle(jointHandle_name);
  } catch (...) {
    ROS_ERROR_STREAM_NAMED(name, "Could not find joint '"
                                     << jointHandle_name << "' in '"
                                     << this->getHardwareInterfaceType()
                                     << "'.");
    return false;
  }

  ROS_DEBUG_STREAM_NAMED(name, "Initialized controller '"
                                   << name << "' with:"
                                   << "\n- Hardware interface type: '"
                                   << this->getHardwareInterfaceType() << "'"
                                   << "\n");

  statePub_.reset(
      new realtime_tools::RealtimePublisher<manipulator_msgs::GripperState>(
          controller_nh, "state", 1));

  cmdSub_ = controller_nh.subscribe("command", 1, &gripperController::commandCB,
                                    this);

  cmdRtBuffer_ = std::make_shared<CmdBuff>();
  // Command - non RT version
  manipulator_msgs::GripperCmd initCmd;
  initCmd.stamp = ros::Time::now();
  initCmd.mode = NORMAL;
  initCmd.des_pos = 0.;
  initCmd.des_vel = 0.;
  initCmd.des_eff = 0.;
  initCmd.des_kp = 0.;
  initCmd.des_kd = 0.;
  cmdRtBuffer_->initRT(initCmd);

  // Hardware interface adapter
  hw_iface_adapter_.init(jointHandle_, controller_nh, cmdRtBuffer_);
  return true;
}

template <class HardwareInterface>
void gripperController<HardwareInterface>::update(const ros::Time &time,
                                                  const ros::Duration &period) {
  double current_position = jointHandle_.getPosition();
  double current_velocity = jointHandle_.getVelocity();
  double current_effort = jointHandle_.getEffort();

  double error_position =
      (cmdRtBuffer_->readFromRT())->des_pos - current_position;
  double error_velocity =
      (cmdRtBuffer_->readFromRT())->des_vel - current_velocity;
  double error_effort = (cmdRtBuffer_->readFromRT())->des_eff - current_velocity;

  if (loopCount_ % 10 == 0) {
    if (statePub_ && statePub_->trylock()) {
      statePub_->msg_.mode = controllerState_;
      statePub_->msg_.stamp = time;
      statePub_->msg_.gripper_pos = current_position;
      statePub_->msg_.gripper_vel = current_velocity;
      statePub_->msg_.gripper_eff = current_effort;
      statePub_->msg_.pos_err = error_position;
      statePub_->msg_.vel_err = error_velocity;
      statePub_->msg_.eff_err = error_effort;
      statePub_->unlockAndPublish();
    }
  }
  loopCount_++;

  // Hardware interface adapter: Generate and send commands
  hw_iface_adapter_.updateCommand(time, period);
}

template <class HardwareInterface>
void gripperController<HardwareInterface>::enforceJointLimits(double &command) {
  // Check that this joint has applicable limits
  if (jointUrdf_->type == urdf::Joint::REVOLUTE ||
      jointUrdf_->type == urdf::Joint::PRISMATIC) {
    if (command > jointUrdf_->limits->upper) // above upper limnit
    {
      command = jointUrdf_->limits->upper;
    } else if (command < jointUrdf_->limits->lower) // below lower limit
    {
      command = jointUrdf_->limits->lower;
    }
  }
}

template <class HardwareInterface>
void gripperController<HardwareInterface>::commandCB(const manipulator_msgs::GripperCmd &msg) {
  cmdRtBuffer_->writeFromNonRT(msg);
}

} // namespace gripper_controller
