//
// Created by lsy on 24-9-23.
//

#include "swingarm_hw/hardware_interface.h"

namespace SwingArm {

bool SwingArmHW::init(ros::NodeHandle &rootNh, ros::NodeHandle &robotHwNh) {
  canManager_ = std::make_shared<device::CanManager>(robotHwNh);

//  registerInterface(&jointStateInterface_);
//  registerInterface(&effortJointInterface_);
//  registerInterface(&positionJointInterface_);
  registerInterface(&robotStateInterface_);
  registerInterface(&imuSensorInterface_);

  registerInterface(&effortActuatorInterface_);
  registerInterface(&actuatorStateInterface_);
  
  setupJoints();
  setupUrdf(rootNh);
  setupImus();
  return true;
}

void SwingArmHW::read(const ros::Time &time, const ros::Duration &period) {
  canManager_->read();
  int jointIndex = 0;
  int imuIndex = 0;
  for (const auto &joint : jointNames_) {
    auto jointDevice = canManager_->getActuatorDevices()[joint];
    jointDatas_[jointIndex].pos = jointDevice->getPosition();
    jointDatas_[jointIndex].vel = jointDevice->getVelocity();
    jointDatas_[jointIndex].tau = jointDevice->getEffort();
    jointIndex++;
  }

  for (const auto &imu : imuNames_) {
    auto imuDevice = canManager_->getSTImuDevices()[imu];
    imuDatas_[imuIndex].angularVel[0] = imuDevice->getAngularVelocity()[0];
    imuDatas_[imuIndex].angularVel[1] = imuDevice->getAngularVelocity()[1];
    imuDatas_[imuIndex].angularVel[2] = imuDevice->getAngularVelocity()[2];
    imuDatas_[imuIndex].linearAcc[0] = imuDevice->getLinearAcceleration()[0];
    imuDatas_[imuIndex].linearAcc[1] = imuDevice->getLinearAcceleration()[1];
    imuDatas_[imuIndex].linearAcc[2] = imuDevice->getLinearAcceleration()[2];
    imuDatas_[imuIndex].ori[0] = imuDevice->getOrientation()[0];
    imuDatas_[imuIndex].ori[1] = imuDevice->getOrientation()[1];
    imuDatas_[imuIndex].ori[2] = imuDevice->getOrientation()[2];
    imuDatas_[imuIndex].ori[3] = imuDevice->getOrientation()[3];
    imuIndex++;
  }
  if (isActuatorSpecified_)
    actuatorToJointState_->propagate();
  for (auto effortJointHandle : effortJointHandles_)
    effortJointHandle.setCommand(0.);
}

void SwingArmHW::write(const ros::Time & /*time*/,
                       const ros::Duration &period) {
  if (isActuatorSpecified_) {
    jointToActuatorEffort_->propagate();

    int jointIndex = 0;
    for (const auto &joint : jointNames_) {
      canManager_->getActuatorDevices()[joint]->setCommand(
          jointDatas_[jointIndex].cmdPos, jointDatas_[jointIndex].cmdVel,
          jointDatas_[jointIndex].cmdKp, jointDatas_[jointIndex].cmdKd,
          jointDatas_[jointIndex].cmdTau);
      jointIndex++;
    }

    effortJointSaturationInterface_.enforceLimits(period);
    effortJointSoftLimitsInterface_.enforceLimits(period);
    jointToActuatorEffort_->propagate();
  }
  canManager_->write();
}

bool SwingArmHW::setupJoints() {
  jointNames_ = canManager_->getActuatorNames();
  int jointIndex = 0;
  for (const auto &joint : jointNames_) {
    hardware_interface::ActuatorStateHandle actuatorHandle(
        joint, &jointDatas_[jointIndex].pos, &jointDatas_[jointIndex].vel,
        &jointDatas_[jointIndex].tau);
    hardware_interface::JointStateHandle jointHandle(
        joint, &jointDatas_[jointIndex].pos, &jointDatas_[jointIndex].vel,
        &jointDatas_[jointIndex].tau);

    actuatorStateInterface_.registerHandle(actuatorHandle);
    jointStateInterface_.registerHandle(jointHandle);

    effortActuatorInterface_.registerHandle(hardware_interface::ActuatorHandle(
        actuatorHandle, &jointDatas_[jointIndex].cmdTau));
    jointIndex++;
  }
  isActuatorSpecified_ = true;

  return true;
}

bool SwingArmHW::setupUrdf(ros::NodeHandle &rootNh) {
  if (!loadUrdf(rootNh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }
  if (!setupTransmission(rootNh)) {
    ROS_ERROR("Error occurred while setting up transmission");
    return false;
  }
  if (!setupJointLimit(rootNh)) {
    ROS_ERROR("Error occurred while setting up joint limit");
    return false;
  }
  return true;
}

bool SwingArmHW::setupImus() {
  imuNames_ = canManager_->getImuNames();
  int imuIndex = 0;
  for (const auto &imu : imuNames_) {
    imuDatas_[imuIndex].oriCov[0] = 0.0012;
    imuDatas_[imuIndex].oriCov[4] = 0.0012;
    imuDatas_[imuIndex].oriCov[8] = 0.0012;

    imuDatas_[imuIndex].angularVelCov[0] = 0.0004;
    imuDatas_[imuIndex].angularVelCov[4] = 0.0004;
    imuDatas_[imuIndex].angularVelCov[8] = 0.0004;

    hardware_interface::ImuSensorHandle imuSensorHandle(
        imu, canManager_->getSTImuDevices()[imu]->getFrameID(),
        imuDatas_[imuIndex].ori, imuDatas_[imuIndex].oriCov,
        imuDatas_[imuIndex].angularVel, imuDatas_[imuIndex].angularVelCov,
        imuDatas_[imuIndex].linearAcc, imuDatas_[imuIndex].linearAccCov);
    imuSensorInterface_.registerHandle(imuSensorHandle);
    imuIndex++;
  }
  return true;
}

bool SwingArmHW::loadUrdf(ros::NodeHandle &rootNh) {
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  rootNh.getParam("robot_description", urdfString_);
  return !urdfString_.empty() && urdfModel_->initString(urdfString_);
}

bool SwingArmHW::setupTransmission(ros::NodeHandle &rootNh) {
  if (!isActuatorSpecified_)
    return true;
  try {
    transmissionLoader_ =
        std::make_unique<transmission_interface::TransmissionInterfaceLoader>(
            this, &robotTransmissions_);
  } catch (const std::invalid_argument &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. "
                     << ex.what());
    return false;
  } catch (const pluginlib::LibraryLoadException &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. "
                     << ex.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }
  if (!transmissionLoader_->load(urdfString_)) {
    return false;
  }
  actuatorToJointState_ =
      robotTransmissions_.get<transmission_interface::ActuatorToJointStateInterface>();
  jointToActuatorEffort_ =
      robotTransmissions_.get<transmission_interface::JointToActuatorEffortInterface>();
  auto effortJointInterface =
      this->get<hardware_interface::EffortJointInterface>();

  std::vector<std::string> names = effortJointInterface->getNames();
  for (const auto &name : names)
    effortJointHandles_.push_back(effortJointInterface->getHandle(name));

  return true;
}

bool SwingArmHW::setupJointLimit(ros::NodeHandle &rootNh) {
  if (!isActuatorSpecified_)
    return true;
  joint_limits_interface::JointLimits jointLimits;
  joint_limits_interface::SoftJointLimits softLimits;
  for (const auto &jointHandle : effortJointHandles_) {
    bool hasJointLimits{}, hasSoftLimits{};
    std::string name = jointHandle.getName();
    urdf::JointConstSharedPtr urdfJoint =
        urdfModel_->getJoint(jointHandle.getName());
    if (urdfJoint == nullptr) {
      ROS_ERROR_STREAM("URDF joint not found " << name);
      return false;
    }
    if (joint_limits_interface::getJointLimits(urdfJoint, jointLimits)) {
      hasJointLimits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has URDF position limits.");
    } else if (urdfJoint->type != urdf::Joint::CONTINUOUS)
      ROS_DEBUG_STREAM("Joint " << name << " does not have a URDF limit.");
    if (joint_limits_interface::getSoftJointLimits(urdfJoint, softLimits)) {
      hasSoftLimits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from URDF.");
    } else
      ROS_DEBUG_STREAM(
          "Joint " << name << " does not have soft joint limits from URDF.");
    if (joint_limits_interface::getJointLimits(jointHandle.getName(), rootNh,
                                               jointLimits)) {
      hasJointLimits = true;
      ROS_DEBUG_STREAM("Joint " << name << " has rosparam position limits.");
    }
    if (joint_limits_interface::getSoftJointLimits(jointHandle.getName(),
                                                   rootNh, softLimits)) {
      hasSoftLimits = true;
      ROS_DEBUG_STREAM("Joint " << name
                                << " has soft joint limits from ROS param.");
    } else
      ROS_DEBUG_STREAM("Joint "
                       << name
                       << " does not have soft joint limits from ROS param.");

    if (jointLimits.has_position_limits) {
      jointLimits.min_position += std::numeric_limits<double>::epsilon();
      jointLimits.max_position -= std::numeric_limits<double>::epsilon();
    }
    if (hasSoftLimits) {
      ROS_DEBUG_STREAM("Using soft saturation limits");
      effortJointSoftLimitsInterface_.registerHandle(
          joint_limits_interface::EffortJointSoftLimitsHandle(
              jointHandle, jointLimits, softLimits));
    } else if (hasJointLimits) {
      ROS_DEBUG_STREAM("Using saturation limits (not soft limits)");
      effortJointSaturationInterface_.registerHandle(
          joint_limits_interface::EffortJointSaturationHandle(jointHandle,
                                                              jointLimits));
    }
  }
  return true;
}

} // namespace SwingArm