//
// Created by lsy on 24-9-23.
//

#include "swingarm_hw/hardware_interface.h"

namespace SwingArm {

bool SwingArmHW::init(ros::NodeHandle &rootNh, ros::NodeHandle &robotHwNh) {
  canManager = std::make_shared<device::CanManager>(robotHwNh);

  setupJoints();
  setupUrdf(rootNh);
  setupImus();

  registerInterface(&jointStateInterface);
  registerInterface(&effortJointInterface);
  registerInterface(&positionJointInterface);
  registerInterface(&robotStateInterface);
  registerInterface(&imuSensorInterface);

  return true;
}

void SwingArmHW::read(const ros::Time &time, const ros::Duration &period) {
  canManager->read();
  int jointIndex = 0;
  int imuIndex = 0;
  for (const auto &joint : jointNames) {
    auto jointDevice = canManager->getActuatorDevices()[joint];
    jointDatas[jointIndex].pos = jointDevice->getPosition();
    jointDatas[jointIndex].vel = jointDevice->getVelocity();
    jointDatas[jointIndex].tau = jointDevice->getEffort();
    jointIndex++;
  }

  for (const auto &imu : imuNames) {
    auto imuDevice = canManager->getSTImuDevices()[imu];
    imuDatas[imuIndex].angularVel[0] = imuDevice->getAngularVelocity()[0];
    imuDatas[imuIndex].angularVel[1] = imuDevice->getAngularVelocity()[1];
    imuDatas[imuIndex].angularVel[2] = imuDevice->getAngularVelocity()[2];
    imuDatas[imuIndex].linearAcc[0] = imuDevice->getLinearAcceleration()[0];
    imuDatas[imuIndex].linearAcc[1] = imuDevice->getLinearAcceleration()[1];
    imuDatas[imuIndex].linearAcc[2] = imuDevice->getLinearAcceleration()[2];
    imuDatas[imuIndex].ori[0] = imuDevice->getOrientation()[0];
    imuDatas[imuIndex].ori[1] = imuDevice->getOrientation()[1];
    imuDatas[imuIndex].ori[2] = imuDevice->getOrientation()[2];
    imuDatas[imuIndex].ori[3] = imuDevice->getOrientation()[3];
    imuIndex++;
  }
  if (isActuatorSpecified)
    actuatorToJointState->propagate();
  for (auto effortJointHandle : effortJointHandles)
    effortJointHandle.setCommand(0.);
}

void SwingArmHW::write(const ros::Time & /*time*/,
                       const ros::Duration &period) {
  if (isActuatorSpecified) {
    jointToActuatorEffort->propagate();

    int jointIndex = 0;
    for (const auto &joint : jointNames) {
      canManager->getActuatorDevices()[joint]->setCommand(
          jointDatas[jointIndex].cmdPos, jointDatas[jointIndex].cmdVel,
          jointDatas[jointIndex].cmdKp, jointDatas[jointIndex].cmdKd,
          jointDatas[jointIndex].cmdTau);
      jointIndex++;
    }

    effortJointSaturationInterface.enforceLimits(period);
    effortJointSoftLimitsInterface.enforceLimits(period);
    jointToActuatorEffort->propagate();
  }
  canManager->write();
}

bool SwingArmHW::setupJoints() {
  jointNames = canManager->getActuatorNames();
  int jointIndex = 0;
  for (const auto &joint : jointNames) {
    hardware_interface::ActuatorStateHandle actuatorHandle(
        joint, &jointDatas[jointIndex].pos, &jointDatas[jointIndex].vel,
        &jointDatas[jointIndex].tau);
    hardware_interface::JointStateHandle jointHandle(
        joint, &jointDatas[jointIndex].pos, &jointDatas[jointIndex].vel,
        &jointDatas[jointIndex].tau);

    actuatorStateInterface.registerHandle(actuatorHandle);
    jointStateInterface.registerHandle(jointHandle);
    effortActuatorInterface.registerHandle(hardware_interface::ActuatorHandle(
        actuatorHandle, &jointDatas[jointIndex].cmdTau));
    jointIndex++;
  }
  isActuatorSpecified = true;

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
  imuNames = canManager->getImuNames();
  int imuIndex = 0;
  for (const auto &imu : imuNames) {
    imuDatas[imuIndex].oriCov[0] = 0.0012;
    imuDatas[imuIndex].oriCov[4] = 0.0012;
    imuDatas[imuIndex].oriCov[8] = 0.0012;

    imuDatas[imuIndex].angularVelCov[0] = 0.0004;
    imuDatas[imuIndex].angularVelCov[4] = 0.0004;
    imuDatas[imuIndex].angularVelCov[8] = 0.0004;

    hardware_interface::ImuSensorHandle imuSensorHandle(
        imu, canManager->getSTImuDevices()[imu]->getFrameID(),
        imuDatas[imuIndex].ori, imuDatas[imuIndex].oriCov,
        imuDatas[imuIndex].angularVel, imuDatas[imuIndex].angularVelCov,
        imuDatas[imuIndex].linearAcc, imuDatas[imuIndex].linearAccCov);
    imuSensorInterface.registerHandle(imuSensorHandle);
    imuIndex++;
  }
  return true;
}

bool SwingArmHW::loadUrdf(ros::NodeHandle &rootNh) {
  std::string urdfStringLocal;
  if (urdfModel == nullptr) {
    urdfModel = std::make_shared<urdf::Model>();
  }
  rootNh.getParam("robot_description", urdfStringLocal);
  return !urdfStringLocal.empty() && urdfModel->initString(urdfStringLocal);
}

bool SwingArmHW::setupTransmission(ros::NodeHandle &rootNh) {
  if (!isActuatorSpecified)
    return true;
  try {
    transmissionLoader =
        std::make_unique<transmission_interface::TransmissionInterfaceLoader>(
            this, &robotTransmissions);
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
  if (!transmissionLoader->load(urdfString)) {
    return false;
  }
  actuatorToJointState =
      robotTransmissions.get<transmission_interface::ActuatorToJointStateInterface>();
  jointToActuatorEffort =
      robotTransmissions.get<transmission_interface::JointToActuatorEffortInterface>();
  auto effortJointInterface =
      this->get<hardware_interface::EffortJointInterface>();

  std::vector<std::string> names = effortJointInterface->getNames();
  for (const auto &name : names)
    effortJointHandles.push_back(effortJointInterface->getHandle(name));

  return true;
}

bool SwingArmHW::setupJointLimit(ros::NodeHandle &rootNh) {
  if (!isActuatorSpecified)
    return true;
  joint_limits_interface::JointLimits jointLimits;
  joint_limits_interface::SoftJointLimits softLimits;
  for (const auto &jointHandle : effortJointHandles) {
    bool hasJointLimits{}, hasSoftLimits{};
    std::string name = jointHandle.getName();
    urdf::JointConstSharedPtr urdfJoint =
        urdfModel->getJoint(jointHandle.getName());
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
      effortJointSoftLimitsInterface.registerHandle(
          joint_limits_interface::EffortJointSoftLimitsHandle(
              jointHandle, jointLimits, softLimits));
    } else if (hasJointLimits) {
      ROS_DEBUG_STREAM("Using saturation limits (not soft limits)");
      effortJointSaturationInterface.registerHandle(
          joint_limits_interface::EffortJointSaturationHandle(jointHandle,
                                                              jointLimits));
    }
  }
  return true;
}

} // namespace SwingArm