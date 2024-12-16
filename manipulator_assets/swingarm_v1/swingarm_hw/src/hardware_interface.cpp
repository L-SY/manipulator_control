//
// Created by lsy on 24-9-23.
//

#include "swingarm_hw/hardware_interface.h"

namespace SwingArm {
bool SwingArmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  canManager_ = std::make_shared<device::CanManager>(robot_hw_nh);

  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&effortJointInterface_);
  registerInterface(&positionJointInterface_);
  registerInterface(&robotStateInterface_);

  setupJoints();
  return true;
}

bool SwingArmHW::loadUrdf(ros::NodeHandle& rootNh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // get the urdf param on param server
  rootNh.getParam("robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

void SwingArmHW::read(const ros::Time& time, const ros::Duration& period) {
  canManager_->read();
  int joint_index = 0;
  for (const auto& joint : urdfModel_->joints_)
  {
    jointData_[joint_index].pos_ = canManager_->getActuatorDevices()[joint.first]->getPosition();
    jointData_[joint_index].vel_ = canManager_->getActuatorDevices()[joint.first]->getVelocity();
    jointData_[joint_index].tau_ = canManager_->getActuatorDevices()[joint.first]->getEffort();
    joint_index ++;
  }
}

void SwingArmHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  int joint_index = 0;
  for (const auto& joint : urdfModel_->joints_)
  {
//    void setCommand(double pos, double vel, double kp, double kd, double effort);
    canManager_->getActuatorDevices()[joint.first]->setCommand(jointData_[joint_index].cmdPos_,jointData_[joint_index].cmdVel_, jointData_[joint_index].cmdKp_, jointData_[joint_index].cmdKd_, jointData_[joint_index].cmdTau_);
    joint_index ++;
  }
}

bool SwingArmHW::setupJoints() {
  int joint_index = 0;
  for (const auto& joint : urdfModel_->joints_)
  {
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[joint_index].pos_, &jointData_[joint_index].vel_,
                                                      &jointData_[joint_index].tau_);
    jointStateInterface_.registerHandle(state_handle);
//    positionJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointData_[joint_index].cmdTau_));
    effortJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointData_[joint_index].cmdTau_));
    joint_index ++;
  }
  return true;
}

}  // namespace SwingArm