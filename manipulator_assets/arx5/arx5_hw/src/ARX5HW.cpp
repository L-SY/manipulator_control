//
// Created by lsy on 24-9-23.
//

#include "arx5_hw/ARX5HW.h"

namespace arx5 {
bool ARX5HW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
//  Init ARX5
  ARX5 = std::make_unique<arx_arm>(CONTROL_MODE);

  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&effortJointInterface_);
  registerInterface(&positionJointInterface_);
//  registerInterface(&robotStateInterface_);

  setupJoints();
  return true;
}

bool ARX5HW::loadUrdf(ros::NodeHandle& rootNh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // get the urdf param on param server
  rootNh.getParam("robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

void ARX5HW::read(const ros::Time& time, const ros::Duration& period) {
  ARX5->get_joint();
  for (int i = 0; i < static_cast<int>(jointName.size()); ++i)
  {
    jointData_[i].pos_ = rv_motor_msg[jointID[i]].angle_actual_rad;
    jointData_[i].vel_ = rv_motor_msg[jointID[i]].speed_actual_rad;
    jointData_[i].tau_ = rv_motor_msg[jointID[i]].current_actual_float;
  }
  CAN_Handlej.arx_1();
}

void ARX5HW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  command cmd;
  if(!ARX5->is_starting){
    cmd = ARX5->get_cmd();
  }
  ARX5->update_real(cmd);
  for (int i = 0; i < static_cast<int>(jointName.size()); ++i)
  {
//    ARX5->ros_control_cur[i] = jointData_[i].cmdTau_;
      ARX5->ros_control_pos[i] = jointData_[i].cmdPos_;
  }
}

bool ARX5HW::setupJoints() {
  for (int i = 0; i < static_cast<int>(jointName.size()); ++i)
  {
    hardware_interface::JointStateHandle state_handle(jointName[i], &jointData_[i].pos_, &jointData_[i].vel_,
                                                      &jointData_[i].tau_);
    jointStateInterface_.registerHandle(state_handle);
    positionJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointData_[i].cmdPos_));
//    effortJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointData_[i].cmdTau_));

  }
  return true;
}

}  // namespace arx5