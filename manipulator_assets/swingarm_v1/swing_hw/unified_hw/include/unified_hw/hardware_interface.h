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
#include <transmission_interface/transmission_interface_loader.h>
#include <unified_hw/hardware_interface/HybridJointInterface.h>
#include <unified_hw/hardware_interface/actuator_calibration_interface.h>
#include <unified_hw/hardware_interface/robot_state_interface.h>

#include <swing_hw_msgs/ActuatorState.h>
#include "unified_hw/can_manager/can_manager.h"
#include "unified_hw/hardware_interface/robot_state_interface.h"
#include "unified_hw/hardware_interface/button_panel_interface.h"

namespace Unified {

class UnifiedHW : public hardware_interface::RobotHW {
public:
  UnifiedHW() = default;

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

  hardware_interface::HybridJointInterface hybridJointInterface_;
  hardware_interface::ActuatorStateInterface actuatorStateInterface_;
  hardware_interface::ActuatorCalibrationInterface actuatorCalibrationInterface_;
  std::vector<hardware_interface::JointHandle> effortJointHandles_;
  transmission_interface::RobotTransmissions robotTransmissions_;
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmissionLoader_;
  transmission_interface::ActuatorToJointStateInterface* actuatorToJointState_{nullptr};

  hardware_interface::EffortActuatorInterface effortActuatorInterface_;
  transmission_interface::JointToActuatorEffortInterface* jointToActuatorEffort_{nullptr};
  joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effortJointSoftLimitsInterface_;

  hardware_interface::PositionActuatorInterface positionActuatorInterface_;
  transmission_interface::JointToActuatorPositionInterface* jointToActuatorPosition_{nullptr};
  joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface_;
  joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface_;

  std::vector<hardware_interface::JointHandle> positionJointHandles_;
  // URDF model of the robot
  std::string urdfString_;                  // for transmission
  std::shared_ptr<urdf::Model> urdfModel_;  // for limit

  std::shared_ptr<device::CanManager> canManager_;
  bool initFlag_{false}, isActuatorSpecified_{false};
  std::vector<std::string> jointNames_, imuNames_;
};

} // namespace Unified
