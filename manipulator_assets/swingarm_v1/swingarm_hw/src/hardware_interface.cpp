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

  setupJoints();
  setupImus();

  registerInterface(&jointStateInterface_);
  registerInterface(&effortJointInterface_);
  registerInterface(&positionJointInterface_);
  registerInterface(&robotStateInterface_);
  registerInterface(&imu_sensor_interface_);

  return true;
}

void SwingArmHW::read(const ros::Time& time, const ros::Duration& period) {
  canManager_->read();
  int joint_index = 0;
  int imu_index = 0;
  for (const auto& joint : jointNames_)
  {
    auto joint_device = canManager_->getActuatorDevices()[joint];
    jointDatas_[joint_index].pos_ = joint_device->getPosition();
    jointDatas_[joint_index].vel_ = joint_device->getVelocity();
    jointDatas_[joint_index].tau_ = joint_device->getEffort();
    joint_index ++;
  }

  for (const auto& imu : imuNames_)
  {
    auto imu_device = canManager_->getSTImuDevices()[imu];
    imuDates_[imu_index].angularVel_[0] = imu_device->getAngularVelocity()[0];
    imuDates_[imu_index].angularVel_[1] = imu_device->getAngularVelocity()[1];
    imuDates_[imu_index].angularVel_[2] = imu_device->getAngularVelocity()[2];
    imuDates_[imu_index].linearAcc_[0] = imu_device->getLinearAcceleration()[0];
    imuDates_[imu_index].linearAcc_[1] = imu_device->getLinearAcceleration()[1];
    imuDates_[imu_index].linearAcc_[2] = imu_device->getLinearAcceleration()[2];
    imuDates_[imu_index].ori_[0] = imu_device->getOrientation()[0];
    imuDates_[imu_index].ori_[1] = imu_device->getOrientation()[1];
    imuDates_[imu_index].ori_[2] = imu_device->getOrientation()[2];
    imuDates_[imu_index].ori_[3] = imu_device->getOrientation()[3];
    imu_index ++;
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

bool SwingArmHW::setupImus() {
  imuNames_ = canManager_->getImuNames();
  int imu_index = 0;
  for (const auto& imu : imuNames_)
  {
    imuDates_[imu_index].oriCov_[0] = 0.0012;
    imuDates_[imu_index].oriCov_[4] = 0.0012;
    imuDates_[imu_index].oriCov_[8] = 0.0012;

    imuDates_[imu_index].angularVelCov_[0] = 0.0004;
    imuDates_[imu_index].angularVelCov_[4] = 0.0004;
    imuDates_[imu_index].angularVelCov_[8] = 0.0004;

    hardware_interface::ImuSensorHandle imu_sensor_handle(
        imu, canManager_->getSTImuDevices()[imu]->getFrameID(),
        imuDates_[imu_index].ori_,
        imuDates_[imu_index].oriCov_,
        imuDates_[imu_index].angularVel_,
        imuDates_[imu_index].angularVelCov_,
        imuDates_[imu_index].linearAcc_,
        imuDates_[imu_index].linearAccCov_);
    imu_sensor_interface_.registerHandle(imu_sensor_handle);
    imu_index ++;
  }
  return true;
}
}  // namespace SwingArm