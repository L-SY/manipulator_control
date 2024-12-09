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
//#include <manipulator_common/interface/hardware_interface/robot_state_interface.h>
#include <hardware_interface/robot_hw.h>

//ARX5 SDK
//#include "App/arm_control.h"
//#include "arm_control/include/App/arm_control.h"
#include "arx5_hw/arx_lib/App/arm_control.h"
#include "arx5_hw/arx_lib/App/keyboard.h"

namespace arx5 {

struct ARXMotorData {
  double pos_, vel_, tau_;                 // state
  double cmdTau_, cmdPos_, cmdVel_, cmdKp_, cmdKd_;  // command
};

class ARX5HW : public hardware_interface::RobotHW {
public:
  ARX5HW() = default;
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

  // Interface
  hardware_interface::JointStateInterface jointStateInterface_;  // NOLINT(misc-non-private-member-variables-in-classes)
  hardware_interface::EffortJointInterface effortJointInterface_;
  hardware_interface::PositionJointInterface positionJointInterface_;
//  hardware_interface::RobotStateInterface robotStateInterface_;

  // URDF model of the robot
  std::shared_ptr<urdf::Model> urdfModel_;  // NOLINT(misc-non-private-member-variables-in-classes)

  bool init_ = false;
  ARXMotorData jointData_[7]{};
  std::vector<std::string> jointName = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};
  std::vector<int> jointID = {0, 1, 3, 4, 5, 6, 7};
  //0 1 keyboard  2 joint_control   4 pose_control
  int CONTROL_MODE=2;
  std::unique_ptr<arx_arm> ARX5;
  can CAN_Handlej;
};

}// namespace arx5
