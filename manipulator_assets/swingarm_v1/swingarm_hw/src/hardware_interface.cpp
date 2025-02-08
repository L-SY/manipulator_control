//
// Created by lsy on 24-9-23.
//

#include "swingarm_hw/hardware_interface.h"

namespace SwingArm {
bool SwingArmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  canManager_ = std::make_shared<device::CanManager>(robot_hw_nh);

//  if (!loadUrdf(root_nh)) {
//    ROS_ERROR("Error occurred while setting up urdf");
//    return false;
//  }

  registerInterface(&jointStateInterface_);
  registerInterface(&effortJointInterface_);
  registerInterface(&positionJointInterface_);
  registerInterface(&robotStateInterface_);

  setupJoints();
  return true;
}

void SwingArmHW::read(const ros::Time& time, const ros::Duration& period) {
  canManager_->read();
  int joint_index = 0;
  for (const auto& joint : jointNames_)
  {
    jointDatas_[joint_index].pos_ = canManager_->getActuatorDevices()[joint]->getPosition();
    jointDatas_[joint_index].vel_ = canManager_->getActuatorDevices()[joint]->getVelocity();
    jointDatas_[joint_index].tau_ = canManager_->getActuatorDevices()[joint]->getEffort();
    joint_index ++;
  }
}

void SwingArmHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  int joint_index = 0;
  for (const auto& joint : jointNames_)
  {
//    void setCommand(double pos, double vel, double kp, double kd, double effort);
    canManager_->getActuatorDevices()[joint]->setCommand(jointDatas_[joint_index].cmdPos_,jointDatas_[joint_index].cmdVel_, jointDatas_[joint_index].cmdKp_, jointDatas_[joint_index].cmdKd_, jointDatas_[joint_index].cmdTau_);
    joint_index ++;
  }
  canManager_->write();
}

bool SwingArmHW::setupJoints() {
  jointNames_ = canManager_->getActuatorNames();
  int joint_index = 0;
  for (const auto& joint : jointNames_)
  {
    hardware_interface::JointStateHandle state_handle(joint, &jointDatas_[joint_index].pos_, &jointDatas_[joint_index].vel_,
                                                      &jointDatas_[joint_index].tau_);
    jointStateInterface_.registerHandle(state_handle);
//    positionJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointDatas_[joint_index].cmdTau_));
    effortJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointDatas_[joint_index].cmdTau_));
    joint_index ++;
  }
  return true;
}

}  // namespace SwingArm