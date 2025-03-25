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

#include <hw_msgs/ActuatorState.h>
#include "swingarm_hw/can_devices/can_manager.h"
#include "swingarm_hw/hardware_interface/robot_state_interface.h"

namespace SwingArm {

struct SwingArmJointData {
  double pos_, vel_, tau_;                 // state
  double cmdTau_, cmdPos_, cmdVel_, cmdKp_, cmdKd_;  // command
};

struct SwingArmImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class SwingArmHW : public hardware_interface::RobotHW {
public:
  SwingArmHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  /** \brief Load urdf of robot from param server.
   *
   * Load urdf of robot from param server.
   *
   * @param rootNh Root node-handle of a ROS node
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& rootNh);

  bool setupJoints();

  bool setupImus();

  // Interface
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::EffortJointInterface effortJointInterface_;
  hardware_interface::PositionJointInterface positionJointInterface_;
  hardware_interface::RobotStateInterface robotStateInterface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;

  // URDF model of the robot
  std::shared_ptr<urdf::Model> urdfModel_;

  std::shared_ptr<device::CanManager> canManager_;
  bool init_ = false;
  SwingArmJointData jointDatas_[8]{};
  SwingArmImuData imuDates_[3]{};
  std::vector<std::string> jointNames_, imuNames_;
};

}// namespace SwingArm
