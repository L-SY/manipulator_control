//
// Created by lsy on 24-9-23.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include "realtime_tools/realtime_publisher.h"
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <swingarm_hw/hardware_interface/actuator_extra_interface.h>
#include <swingarm_hw/hardware_interface/robot_state_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <swingarm_hw/hardware_interface/HybridJointInterface.h>

#include <swing_hw_msgs/ActuatorState.h>
#include "swingarm_hw/can_devices/can_manager.h"
#include "swingarm_hw/hardware_interface/robot_state_interface.h"
#include "swingarm_hw/hardware_interface/button_panel_interface.h"

namespace SwingArm {

struct SwingArmJointData {
  double pos, vel, tau;                 // state
  double cmdTau, cmdPos, cmdVel, cmdKp, cmdKd;  // command
};

struct SwingArmImuData {
  double ori[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov[9];   // NOLINT(modernize-avoid-c-arrays)
};

class SwingArmHW : public hardware_interface::RobotHW {
public:
  SwingArmHW() = default;

  bool init(ros::NodeHandle& rootNh, ros::NodeHandle& robotHwNh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  bool setupUrdf(ros::NodeHandle &rootNh);
  bool setupTransmission(ros::NodeHandle &rootNh);
  bool setupJointLimit(ros::NodeHandle &rootNh);

private:
  bool loadUrdf(ros::NodeHandle& rootNh);
  bool setupJoints();
  bool setupImus();
  bool setupButtonPanels();

  // ROS Interface
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::EffortJointInterface effortJointInterface_;
  hardware_interface::PositionJointInterface positionJointInterface_;
  hardware_interface::RobotStateInterface robotStateInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;

  // Personal Interface
  hardware_interface::ButtonPanelInterface buttonPanelInterface_;

  // For transmission
  hardware_interface::EffortActuatorInterface effortActuatorInterface_;
  hardware_interface::HybridJointInterface hybridJointInterface_;
  hardware_interface::ActuatorStateInterface actuatorStateInterface_;
  hardware_interface::ActuatorExtraInterface actuatorExtraInterface_;
  std::vector<hardware_interface::JointHandle> effortJointHandles_;
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmissionLoader_;
  transmission_interface::RobotTransmissions robotTransmissions_;
  transmission_interface::ActuatorToJointStateInterface* actuatorToJointState_{nullptr};
  transmission_interface::JointToActuatorEffortInterface* jointToActuatorEffort_{nullptr};
  joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effortJointSoftLimitsInterface_;

  // URDF model of the robot
  std::string urdfString_;                  // for transmission
  std::shared_ptr<urdf::Model> urdfModel_;  // for limit

  std::shared_ptr<device::CanManager> canManager_;
  bool initFlag_{false}, isActuatorSpecified_{false};
  SwingArmJointData jointDatas_[8]{};
  SwingArmImuData imuDatas_[3]{};
  std::vector<std::string> jointNames_, imuNames_;
};

} // namespace SwingArm
