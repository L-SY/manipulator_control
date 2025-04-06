//
// Created by lsy on 24-9-23.
//

#include "swingarm_hw/hardware_interface.h"

namespace SwingArm {

bool SwingArmHW::init(ros::NodeHandle &rootNh, ros::NodeHandle &robotHwNh) {
  try {
    canManager_ = std::make_shared<device::CanManager>(robotHwNh);

    registerInterface(&robotStateInterface_);
    registerInterface(&imuSensorInterface_);
    registerInterface(&buttonPanelInterface_);
    registerInterface(&actuatorStateInterface_);

    bool setupJointsSuccess = setupJoints();
    ROS_INFO_STREAM("Setup joints: " << (setupJointsSuccess ? "success" : "failed but continuing"));

    bool setupUrdfSuccess = setupUrdf(rootNh);
    ROS_INFO_STREAM("Setup URDF: " << (setupUrdfSuccess ? "success" : "failed but continuing"));

    bool setupImusSuccess = setupImus();
    ROS_INFO_STREAM("Setup IMUs: " << (setupImusSuccess ? "success" : "failed but continuing"));

    bool setupButtonPanelsSuccess = setupButtonPanels();
    ROS_INFO_STREAM("Setup button panels: " << (setupButtonPanelsSuccess ? "success" : "failed but continuing"));

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in init: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in init");
    return false;
  }
}


void SwingArmHW::read(const ros::Time &time, const ros::Duration &period) {
  try {
    canManager_->read();

    if (!jointNames_.empty()) {
      int jointIndex = 0;
      for (const auto &joint : jointNames_) {
        if (jointIndex >= 8) {
          ROS_WARN_STREAM("Joint index out of bounds, skipping joint: " << joint);
          break;
        }

        try {
          if (canManager_->getActuatorDevices().find(joint) != canManager_->getActuatorDevices().end()) {
            auto jointDevice = canManager_->getActuatorDevices()[joint];
            if (jointDevice) {
              jointDatas_[jointIndex].pos = jointDevice->getPosition();
              jointDatas_[jointIndex].vel = jointDevice->getVelocity();
              jointDatas_[jointIndex].tau = jointDevice->getEffort();
            } else {
              ROS_WARN_STREAM("Joint device is null for joint: " << joint);
            }
          } else {
            ROS_WARN_STREAM("Joint not found in actuator devices: " << joint);
          }
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("Exception reading joint " << joint << ": " << e.what());
        } catch (...) {
          ROS_ERROR_STREAM("Unknown exception reading joint " << joint);
        }

        jointIndex++;
      }
    }

    if (!imuNames_.empty()) {
      int imuIndex = 0;
      for (const auto &imu : imuNames_) {
        if (imuIndex >= 3) {
          ROS_WARN_STREAM("IMU index out of bounds, skipping IMU: " << imu);
          break;
        }

        try {
          if (canManager_->getSTImuDevices().find(imu) != canManager_->getSTImuDevices().end()) {
            auto imuDevice = canManager_->getSTImuDevices()[imu];
            if (imuDevice) {
              auto angVel = imuDevice->getAngularVelocity();
              if (angVel.size() >= 3) {
                imuDatas_[imuIndex].angularVel[0] = angVel[0];
                imuDatas_[imuIndex].angularVel[1] = angVel[1];
                imuDatas_[imuIndex].angularVel[2] = angVel[2];
              } else {
                ROS_WARN_STREAM("Angular velocity vector size mismatch for IMU: " << imu);
              }

              auto linAcc = imuDevice->getLinearAcceleration();
              if (linAcc.size() >= 3) {
                imuDatas_[imuIndex].linearAcc[0] = linAcc[0];
                imuDatas_[imuIndex].linearAcc[1] = linAcc[1];
                imuDatas_[imuIndex].linearAcc[2] = linAcc[2];
              } else {
                ROS_WARN_STREAM("Linear acceleration vector size mismatch for IMU: " << imu);
              }

              auto ori = imuDevice->getOrientation();
              if (ori.size() >= 4) {
                imuDatas_[imuIndex].ori[0] = ori[0];
                imuDatas_[imuIndex].ori[1] = ori[1];
                imuDatas_[imuIndex].ori[2] = ori[2];
                imuDatas_[imuIndex].ori[3] = ori[3];
              } else {
                ROS_WARN_STREAM("Orientation vector size mismatch for IMU: " << imu);
              }
            } else {
              ROS_WARN_STREAM("IMU device is null for IMU: " << imu);
            }
          } else {
            ROS_WARN_STREAM("IMU not found in ST IMU devices: " << imu);
          }
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("Exception reading IMU " << imu << ": " << e.what());
        } catch (...) {
          ROS_ERROR_STREAM("Unknown exception reading IMU " << imu);
        }

        imuIndex++;
      }
    }

    if (isActuatorSpecified_ && actuatorToJointState_) {
      try {
        actuatorToJointState_->propagate();
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception propagating actuator to joint state: " << e.what());
      } catch (...) {
        ROS_ERROR("Unknown exception propagating actuator to joint state");
      }
    }

    for (auto &effortJointHandle : effortJointHandles_) {
      try {
        effortJointHandle.setCommand(0.);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception resetting effort command for joint "
                         << effortJointHandle.getName() << ": " << e.what());
      } catch (...) {
        ROS_ERROR_STREAM("Unknown exception resetting effort command for joint "
                         << effortJointHandle.getName());
      }
    }

    for (auto &positionJointHandle : positionJointHandles_) {
      try {
        positionJointHandle.setCommand(positionJointHandle.getPosition());
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception resetting position command for joint "
                         << positionJointHandle.getName() << ": " << e.what());
      } catch (...) {
        ROS_ERROR_STREAM("Unknown exception resetting position command for joint "
                         << positionJointHandle.getName());
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in read: " << e.what());
  } catch (...) {
    ROS_ERROR("Unknown exception in read");
  }
}


void SwingArmHW::write(const ros::Time & /*time*/, const ros::Duration &period) {
  try {
    if (isActuatorSpecified_) {
      if (jointToActuatorEffort_) {
        try {
          jointToActuatorEffort_->propagate();
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("Exception propagating joint to actuator effort: " << e.what());
        } catch (...) {
          ROS_ERROR("Unknown exception propagating joint to actuator effort");
        }
      }

      if (!jointNames_.empty()) {
        int jointIndex = 0;
        for (const auto &joint : jointNames_) {
          if (jointIndex >= 8) {
            ROS_WARN_STREAM("Joint index out of bounds, skipping joint: " << joint);
            break;
          }

          try {
            if (canManager_->getActuatorDevices().find(joint) != canManager_->getActuatorDevices().end()) {
              auto jointDevice = canManager_->getActuatorDevices()[joint];
              if (jointDevice) {
                jointDevice->setCommand(
                    jointDatas_[jointIndex].cmdPos,
                    jointDatas_[jointIndex].cmdVel,
                    jointDatas_[jointIndex].cmdKp,
                    jointDatas_[jointIndex].cmdKd,
                    jointDatas_[jointIndex].cmdTau);
              } else {
                ROS_WARN_STREAM("Joint device is null for joint: " << joint);
              }
            } else {
              ROS_WARN_STREAM("Joint not found in actuator devices: " << joint);
            }
          } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Exception setting command for joint " << joint << ": " << e.what());
          } catch (...) {
            ROS_ERROR_STREAM("Unknown exception setting command for joint " << joint);
          }

          jointIndex++;
        }
      }

      try {
        effortJointSaturationInterface_.enforceLimits(period);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception enforcing effort saturation limits: " << e.what());
      } catch (...) {
        ROS_ERROR("Unknown exception enforcing effort saturation limits");
      }

      try {
        effortJointSoftLimitsInterface_.enforceLimits(period);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception enforcing effort soft limits: " << e.what());
      } catch (...) {
        ROS_ERROR("Unknown exception enforcing effort soft limits");
      }

      if (jointToActuatorEffort_) {
        try {
          jointToActuatorEffort_->propagate();
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("Exception propagating joint to actuator effort (second time): " << e.what());
        } catch (...) {
          ROS_ERROR("Unknown exception propagating joint to actuator effort (second time)");
        }
      }

      try {
        positionJointSaturationInterface_.enforceLimits(period);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception enforcing position saturation limits: " << e.what());
      } catch (...) {
        ROS_ERROR("Unknown exception enforcing position saturation limits");
      }

      try {
        positionJointSoftLimitsInterface_.enforceLimits(period);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception enforcing position soft limits: " << e.what());
      } catch (...) {
        ROS_ERROR("Unknown exception enforcing position soft limits");
      }

      if (jointToActuatorPosition_) {
        try {
          jointToActuatorPosition_->propagate();
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("Exception propagating joint to actuator position: " << e.what());
        } catch (...) {
          ROS_ERROR("Unknown exception propagating joint to actuator position");
        }
      }
    }

    try {
      canManager_->write();
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Exception in canManager write: " << e.what());
    } catch (...) {
      ROS_ERROR("Unknown exception in canManager write");
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in write: " << e.what());
  } catch (...) {
    ROS_ERROR("Unknown exception in write");
  }
}

bool SwingArmHW::setupJoints() {
  try {
    jointNames_ = canManager_->getActuatorNames();

    if (jointNames_.empty()) {
      ROS_WARN("No actuator joints found");
      isActuatorSpecified_ = false;
      return true;
    }

    int jointIndex = 0;
    for (const auto &joint : jointNames_) {
      if (jointIndex >= 8) {
        ROS_WARN_STREAM("Reached maximum number of joints (8). Ignoring joint: " << joint);
        continue;
      }

      try {
        jointDatas_[jointIndex].cmdTau = 0.0;
        jointDatas_[jointIndex].cmdPos = 0.0;
        jointDatas_[jointIndex].cmdVel = 0.0;
        jointDatas_[jointIndex].cmdKp = 0.0;
        jointDatas_[jointIndex].cmdKd = 0.0;

        hardware_interface::ActuatorStateHandle actuatorHandle(
            joint, &jointDatas_[jointIndex].pos, &jointDatas_[jointIndex].vel,
            &jointDatas_[jointIndex].tau);

        actuatorStateInterface_.registerHandle(actuatorHandle);

        hardware_interface::JointStateHandle jointHandle(
            joint, &jointDatas_[jointIndex].pos, &jointDatas_[jointIndex].vel,
            &jointDatas_[jointIndex].tau);

        jointStateInterface_.registerHandle(jointHandle);

        effortActuatorInterface_.registerHandle(hardware_interface::ActuatorHandle(
            actuatorHandle, &jointDatas_[jointIndex].cmdTau));

        positionActuatorInterface_.registerHandle(hardware_interface::ActuatorHandle(
            actuatorHandle, &jointDatas_[jointIndex].cmdPos));

        hybridJointInterface_.registerHandle(hardware_interface::HybridJointHandle(
            jointHandle, &jointDatas_[jointIndex].cmdPos, &jointDatas_[jointIndex].cmdVel,
            &jointDatas_[jointIndex].cmdTau, &jointDatas_[jointIndex].cmdKp, &jointDatas_[jointIndex].cmdKd));

        jointIndex++;
      }
      catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception setting up joint " << joint << ": " << e.what());
      }
    }

    if (jointIndex > 0) {
      isActuatorSpecified_ = true;
      registerInterface(&effortActuatorInterface_);
      registerInterface(&positionActuatorInterface_);
      registerInterface(&hybridJointInterface_);
    } else {
      isActuatorSpecified_ = false;
    }

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in setupJoints: " << e.what());
    isActuatorSpecified_ = false;
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in setupJoints");
    isActuatorSpecified_ = false;
    return false;
  }
}

bool SwingArmHW::setupUrdf(ros::NodeHandle &rootNh) {
  try {
    if (!isActuatorSpecified_) {
      ROS_INFO("No actuators specified, skipping URDF setup");
      return true;
    }

    if (!loadUrdf(rootNh)) {
      ROS_WARN("Failed to load URDF, skipping transmission and joint limit setup");
      return false;
    }

    bool transmissionSuccess = setupTransmission(rootNh);
    if (!transmissionSuccess) {
      ROS_WARN("Failed to setup transmission, but continuing");
    }

    bool jointLimitSuccess = setupJointLimit(rootNh);
    if (!jointLimitSuccess) {
      ROS_WARN("Failed to setup joint limits, but continuing");
    }

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in setupUrdf: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in setupUrdf");
    return false;
  }
}

bool SwingArmHW::setupImus() {
  try {
    imuNames_ = canManager_->getImuNames();

    if (imuNames_.empty()) {
      ROS_INFO("No IMUs found");
      return true;
    }

    int imuIndex = 0;
    for (const auto &imu : imuNames_) {
      if (imuIndex >= 3) {
        ROS_WARN_STREAM("Reached maximum number of IMUs (3). Ignoring IMU: " << imu);
        continue;
      }

      try {
        for (int i = 0; i < 4; i++) imuDatas_[imuIndex].ori[i] = 0.0;
        for (int i = 0; i < 3; i++) imuDatas_[imuIndex].angularVel[i] = 0.0;
        for (int i = 0; i < 3; i++) imuDatas_[imuIndex].linearAcc[i] = 0.0;

        for (int i = 0; i < 9; i++) imuDatas_[imuIndex].oriCov[i] = 0.0;
        for (int i = 0; i < 9; i++) imuDatas_[imuIndex].angularVelCov[i] = 0.0;
        for (int i = 0; i < 9; i++) imuDatas_[imuIndex].linearAccCov[i] = 0.0;

        imuDatas_[imuIndex].oriCov[0] = 0.0012;
        imuDatas_[imuIndex].oriCov[4] = 0.0012;
        imuDatas_[imuIndex].oriCov[8] = 0.0012;

        imuDatas_[imuIndex].angularVelCov[0] = 0.0004;
        imuDatas_[imuIndex].angularVelCov[4] = 0.0004;
        imuDatas_[imuIndex].angularVelCov[8] = 0.0004;

        // 确保IMU设备存在
        if (canManager_->getSTImuDevices().find(imu) == canManager_->getSTImuDevices().end()) {
          ROS_WARN_STREAM("IMU device not found: " << imu);
          continue;
        }

        std::string frameId = canManager_->getSTImuDevices()[imu]->getFrameID();
        if (frameId.empty()) {
          frameId = "imu_" + imu + "_frame";
          ROS_WARN_STREAM("IMU " << imu << " has no frame ID, using default: " << frameId);
        }

        hardware_interface::ImuSensorHandle imuSensorHandle(
            imu, frameId,
            imuDatas_[imuIndex].ori, imuDatas_[imuIndex].oriCov,
            imuDatas_[imuIndex].angularVel, imuDatas_[imuIndex].angularVelCov,
            imuDatas_[imuIndex].linearAcc, imuDatas_[imuIndex].linearAccCov);

        imuSensorInterface_.registerHandle(imuSensorHandle);
        imuIndex++;
      }
      catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception setting up IMU " << imu << ": " << e.what());
      }
    }

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in setupImus: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in setupImus");
    return false;
  }
}


bool SwingArmHW::setupButtonPanels() {
  try {
    std::vector<std::string> buttonPanelNames = canManager_->getButtonPanelNames();

    if (buttonPanelNames.empty()) {
      ROS_INFO("No button panels found");
      return true;
    }

    for (const auto & buttonPanelName : buttonPanelNames) {
      try {
        if (canManager_->getButtonPanelDevices().find(buttonPanelName) == canManager_->getButtonPanelDevices().end()) {
          ROS_WARN_STREAM("Button panel device not found: " << buttonPanelName);
          continue;
        }

        hardware_interface::ButtonPanelHandle buttonPanelHandle(
            buttonPanelName,
            &(canManager_->getButtonPanelDevices()[buttonPanelName]->button1_pressed_),
            &(canManager_->getButtonPanelDevices()[buttonPanelName]->button2_pressed_));

        buttonPanelInterface_.registerHandle(buttonPanelHandle);
      }
      catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception setting up button panel " << buttonPanelName << ": " << e.what());
      }
    }

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in setupButtonPanels: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in setupButtonPanels");
    return false;
  }
}


bool SwingArmHW::loadUrdf(ros::NodeHandle &rootNh) {
  try {
    if (urdfModel_ == nullptr) {
      urdfModel_ = std::make_shared<urdf::Model>();
    }

    if (!rootNh.getParam("robot_description", urdfString_)) {
      ROS_WARN("Failed to get robot_description parameter");
      return false;
    }

    if (urdfString_.empty()) {
      ROS_WARN("Empty URDF string");
      return false;
    }

    if (!urdfModel_->initString(urdfString_)) {
      ROS_WARN("Failed to parse URDF");
      return false;
    }

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in loadUrdf: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in loadUrdf");
    return false;
  }
}


bool SwingArmHW::setupTransmission(ros::NodeHandle &rootNh) {
  try {
    if (!isActuatorSpecified_) {
      ROS_INFO("No actuators specified, skipping transmission setup");
      return true;
    }

    if (urdfString_.empty()) {
      ROS_WARN("Empty URDF string, skipping transmission setup");
      return false;
    }

    try {
      transmissionLoader_ =
          std::make_unique<transmission_interface::TransmissionInterfaceLoader>(
              this, &robotTransmissions_);
    }
    catch (const std::invalid_argument &ex) {
      ROS_ERROR_STREAM("Failed to create transmission interface loader: " << ex.what());
      return false;
    }
    catch (const pluginlib::LibraryLoadException &ex) {
      ROS_ERROR_STREAM("Failed to create transmission interface loader: " << ex.what());
      return false;
    }
    catch (...) {
      ROS_ERROR("Unknown exception when creating transmission interface loader");
      return false;
    }

    if (!transmissionLoader_->load(urdfString_)) {
      ROS_WARN("Failed to load transmissions from URDF");
      return false;
    }

    try {
      actuatorToJointState_ =
          robotTransmissions_
              .get<transmission_interface::ActuatorToJointStateInterface>();
      if (!actuatorToJointState_) {
        ROS_WARN("ActuatorToJointState interface not found");
      }
    }
    catch (...) {
      ROS_WARN("Failed to get ActuatorToJointState interface");
      actuatorToJointState_ = nullptr;
    }

    try {
      jointToActuatorEffort_ =
          robotTransmissions_
              .get<transmission_interface::JointToActuatorEffortInterface>();
      if (!jointToActuatorEffort_) {
        ROS_WARN("JointToActuatorEffort interface not found");
      }
    }
    catch (...) {
      ROS_WARN("Failed to get JointToActuatorEffort interface");
      jointToActuatorEffort_ = nullptr;
    }

    try {
      jointToActuatorPosition_ =
          robotTransmissions_
              .get<transmission_interface::JointToActuatorPositionInterface>();
      if (!jointToActuatorPosition_) {
        ROS_WARN("JointToActuatorPosition interface not found");
      }
    }
    catch (...) {
      ROS_WARN("Failed to get JointToActuatorPosition interface");
      jointToActuatorPosition_ = nullptr;
    }

    try {
      auto effortJointInterface = this->get<hardware_interface::EffortJointInterface>();
      if (effortJointInterface) {
        std::vector<std::string> names = effortJointInterface->getNames();
        for (const auto &name : names) {
          try {
            effortJointHandles_.push_back(effortJointInterface->getHandle(name));
          }
          catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_WARN_STREAM("Could not get effort joint handle for " << name << ": " << ex.what());
          }
        }
      }
    }
    catch (...) {
      ROS_WARN("Failed to get effort joint handles");
    }

    try {
      auto positionJointInterface = this->get<hardware_interface::PositionJointInterface>();
      if (positionJointInterface) {
        std::vector<std::string> names = positionJointInterface->getNames();
        for (const auto &name : names) {
          try {
            positionJointHandles_.push_back(positionJointInterface->getHandle(name));
          }
          catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_WARN_STREAM("Could not get position joint handle for " << name << ": " << ex.what());
          }
        }
      }
    }
    catch (...) {
      ROS_WARN("Failed to get position joint handles");
    }

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in setupTransmission: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in setupTransmission");
    return false;
  }
}

bool SwingArmHW::setupJointLimit(ros::NodeHandle &rootNh) {
  try {
    if (!isActuatorSpecified_) {
      ROS_INFO("No actuators specified, skipping joint limit setup");
      return true;
    }

    if (!urdfModel_) {
      ROS_WARN("No valid URDF model, skipping joint limit setup");
      return false;
    }

    joint_limits_interface::JointLimits jointLimits;
    joint_limits_interface::SoftJointLimits softLimits;

    for (const auto &jointHandle : effortJointHandles_) {
      try {
        bool hasJointLimits = false, hasSoftLimits = false;
        std::string name = jointHandle.getName();

        urdf::JointConstSharedPtr urdfJoint = urdfModel_->getJoint(name);
        if (!urdfJoint) {
          ROS_WARN_STREAM("URDF joint not found: " << name);
          continue;
        }

        if (joint_limits_interface::getJointLimits(urdfJoint, jointLimits)) {
          hasJointLimits = true;
          ROS_DEBUG_STREAM("Joint " << name << " has URDF position limits.");
        } else if (urdfJoint->type != urdf::Joint::CONTINUOUS) {
          ROS_DEBUG_STREAM("Joint " << name << " does not have a URDF limit.");
        }

        if (joint_limits_interface::getSoftJointLimits(urdfJoint, softLimits)) {
          hasSoftLimits = true;
          ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from URDF.");
        } else {
          ROS_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from URDF.");
        }

        if (joint_limits_interface::getJointLimits(name, rootNh, jointLimits)) {
          hasJointLimits = true;
          ROS_DEBUG_STREAM("Joint " << name << " has rosparam position limits.");
        }

        if (joint_limits_interface::getSoftJointLimits(name, rootNh, softLimits)) {
          hasSoftLimits = true;
          ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from ROS param.");
        } else {
          ROS_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from ROS param.");
        }

        if (jointLimits.has_position_limits) {
          jointLimits.min_position += std::numeric_limits<double>::epsilon();
          jointLimits.max_position -= std::numeric_limits<double>::epsilon();
        }

        if (hasSoftLimits) {
          ROS_DEBUG_STREAM("Using soft saturation limits for " << name);
          effortJointSoftLimitsInterface_.registerHandle(
              joint_limits_interface::EffortJointSoftLimitsHandle(
                  jointHandle, jointLimits, softLimits));
        } else if (hasJointLimits) {
          ROS_DEBUG_STREAM("Using saturation limits for " << name);
          effortJointSaturationInterface_.registerHandle(
              joint_limits_interface::EffortJointSaturationHandle(
                  jointHandle, jointLimits));
        }
      }
      catch (const std::exception& e) {
        ROS_WARN_STREAM("Exception processing joint limits for " << jointHandle.getName() << ": " << e.what());
      }
    }

    for (const auto &jointHandle : positionJointHandles_) {
      try {
        bool hasJointLimits = false, hasSoftLimits = false;
        std::string name = jointHandle.getName();

        urdf::JointConstSharedPtr urdfJoint = urdfModel_->getJoint(name);
        if (!urdfJoint) {
          ROS_WARN_STREAM("URDF joint not found: " << name);
          continue;
        }

        if (joint_limits_interface::getJointLimits(urdfJoint, jointLimits)) {
          hasJointLimits = true;
          ROS_DEBUG_STREAM("Joint " << name << " has URDF position limits.");
        } else if (urdfJoint->type != urdf::Joint::CONTINUOUS) {
          ROS_DEBUG_STREAM("Joint " << name << " does not have a URDF limit.");
        }

        if (joint_limits_interface::getSoftJointLimits(urdfJoint, softLimits)) {
          hasSoftLimits = true;
          ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from URDF.");
        } else {
          ROS_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from URDF.");
        }

        if (joint_limits_interface::getJointLimits(name, rootNh, jointLimits)) {
          hasJointLimits = true;
          ROS_DEBUG_STREAM("Joint " << name << " has rosparam position limits.");
        }

        if (joint_limits_interface::getSoftJointLimits(name, rootNh, softLimits)) {
          hasSoftLimits = true;
          ROS_DEBUG_STREAM("Joint " << name << " has soft joint limits from ROS param.");
        } else {
          ROS_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from ROS param.");
        }

        if (jointLimits.has_position_limits) {
          jointLimits.min_position += std::numeric_limits<double>::epsilon();
          jointLimits.max_position -= std::numeric_limits<double>::epsilon();
        }

        if (hasSoftLimits) {
          ROS_DEBUG_STREAM("Using soft position saturation limits for " << name);
          positionJointSoftLimitsInterface_.registerHandle(
              joint_limits_interface::PositionJointSoftLimitsHandle(
                  jointHandle, jointLimits, softLimits));
        } else if (hasJointLimits) {
          ROS_DEBUG_STREAM("Using position saturation limits for " << name);
          positionJointSaturationInterface_.registerHandle(
              joint_limits_interface::PositionJointSaturationHandle(
                  jointHandle, jointLimits));
        }
      }
      catch (const std::exception& e) {
        ROS_WARN_STREAM("Exception processing position limits for " << jointHandle.getName() << ": " << e.what());
      }
    }

    return true;
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in setupJointLimit: " << e.what());
    return false;
  }
  catch (...) {
    ROS_ERROR("Unknown exception in setupJointLimit");
    return false;
  }
}

} // namespace SwingArm