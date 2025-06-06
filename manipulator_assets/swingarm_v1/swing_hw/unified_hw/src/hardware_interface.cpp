//
// Created by lsy on 24-9-23.
//

#include "unified_hw/hardware_interface.h"

namespace Unified {

bool UnifiedHW::init(ros::NodeHandle &rootNh, ros::NodeHandle &robotHwNh) {
  try {
    canManager_ = std::make_shared<device::CanManager>(robotHwNh);

    registerInterface(&robotStateInterface_);
//    registerInterface(&imuSensorInterface_);
//    registerInterface(&buttonPanelInterface_);
    registerInterface(&actuatorStateInterface_);

    bool setupJointsSuccess = setupJoints();
    ROS_INFO_STREAM("Setup joints: " << (setupJointsSuccess ? "success" : "failed but continuing"));

    bool setupUrdfSuccess = setupUrdf(rootNh);
    ROS_INFO_STREAM("Setup URDF: " << (setupUrdfSuccess ? "success" : "failed but continuing"));

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


void UnifiedHW::read(const ros::Time &time, const ros::Duration &period) {
  try {
    canManager_->read();

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

void UnifiedHW::write(const ros::Time & /*time*/, const ros::Duration &period) {
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

      if (jointToActuatorPosition_) {
        try {
          jointToActuatorPosition_->propagate();
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("Exception propagating joint to actuator position: " << e.what());
        } catch (...) {
          ROS_ERROR("Unknown exception propagating joint to actuator position");
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

bool UnifiedHW::setupJoints() {
  try {
    const auto& actuator_maps = canManager_->getActuatorDevices();
    if (actuator_maps.empty()) {
      ROS_WARN("No actuator joints found");
      isActuatorSpecified_ = false;
      return true;
    }
    else{
      isActuatorSpecified_ = true;
      registerInterface(&effortActuatorInterface_);
      registerInterface(&positionActuatorInterface_);
      registerInterface(&hybridJointInterface_);
    }

    for (const auto& actuator_pair : actuator_maps) {
      const std::string& joint = actuator_pair.first;
      auto actuator = actuator_pair.second;
      if (!actuator) continue;

      try {
        hardware_interface::ActuatorStateHandle actuatorHandle(
            joint,
            actuator->getPositionPtr(),
            actuator->getVelocityPtr(),
            actuator->getEffortPtr()
        );
        actuatorStateInterface_.registerHandle(actuatorHandle);

        effortActuatorInterface_.registerHandle(
            hardware_interface::ActuatorHandle(
                actuatorHandle,
                actuator->getCmdEffortPtr()
                    )
        );

        positionActuatorInterface_.registerHandle(
            hardware_interface::ActuatorHandle(
                actuatorHandle,
                actuator->getCmdPosPtr()
                    )
        );

//        hybridJointInterface_.registerHandle(
//            hardware_interface::HybridJointHandle(
//                jointHandle,
//                actuator->getCmdPosPtr(),
//                actuator->getCmdVelPtr(),
//                actuator->getCmdKpPtr(),
//                actuator->getCmdKdPtr(),
//                actuator->getCmdEffortPtr()
//                    )
//        );
      }
      catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception setting up joint " << joint << ": " << e.what());
      }
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

bool UnifiedHW::setupUrdf(ros::NodeHandle &rootNh) {
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

bool UnifiedHW::loadUrdf(ros::NodeHandle &rootNh) {
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

bool UnifiedHW::setupTransmission(ros::NodeHandle &rootNh) {
  try {
    if (!isActuatorSpecified_) {
      ROS_INFO("No actuators specified, skipping transmission setup");
      return true;
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

bool UnifiedHW::setupJointLimit(ros::NodeHandle &rootNh) {
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

} // namespace Unified