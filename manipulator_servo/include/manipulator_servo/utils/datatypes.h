//
// Created by lsy on 24-8-4.
//

// ref: https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo

#pragma once

#include "ros/ros.h"
#include <string>
#include <tf2_eigen/tf2_eigen.h>
#include <unordered_map>
#include <variant>

namespace manipulator_servo
{

struct ServoParameters
{
  std::string joint_topic;
  std::string cartesian_command_in_topic;
  std::string robot_link_command_frame;
  std::string command_out_topic;
  std::string planning_frame;
  std::string ee_frame_name;
  std::string status_topic;
  std::string joint_command_in_topic;
  std::string command_in_type;
  std::string command_out_type;
  double linear_scale;
  double rotational_scale;
  double joint_scale;
  double lower_singularity_threshold;
  double hard_stop_singularity_threshold;
  double low_pass_filter_coeff;
  double publish_period;
  double incoming_command_timeout;
  double joint_limit_margin;
  int num_outgoing_halt_msgs_to_publish;
  bool use_gazebo;
  bool publish_joint_positions;
  bool publish_joint_velocities;
  bool publish_joint_accelerations;
  bool low_latency_mode;
  // Collision checking
  bool check_collisions;
  std::string collision_check_type;
  double collision_check_rate;
  double scene_collision_proximity_threshold;
  double self_collision_proximity_threshold;
  double collision_distance_safety_factor;
  double min_allowable_collision_distance;
};

enum class StatusCode : int8_t
{
  INVALID = -1,
  NO_WARNING = 0,
  DECELERATE_FOR_APPROACHING_SINGULARITY = 1,
  HALT_FOR_SINGULARITY = 2,
  DECELERATE_FOR_LEAVING_SINGULARITY = 3,
  DECELERATE_FOR_COLLISION = 4,
  HALT_FOR_COLLISION = 5,
  JOINT_BOUND = 6
};

const std::unordered_map<StatusCode, std::string> SERVO_STATUS_CODE_MAP(
    { { StatusCode::INVALID, "Invalid" },
     { StatusCode::NO_WARNING, "No warnings" },
     { StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY, "Moving closer to a singularity, decelerating" },
     { StatusCode::HALT_FOR_SINGULARITY, "Very close to a singularity, emergency stop" },
     { StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY, "Moving away from a singularity, decelerating" },
     { StatusCode::DECELERATE_FOR_COLLISION, "Close to a collision, decelerating" },
     { StatusCode::HALT_FOR_COLLISION, "Collision detected, emergency stop" },
     { StatusCode::JOINT_BOUND, "Close to a joint bound (position or velocity), halting" } });

// The datatype that specifies the type of command that servo should expect.
enum class CommandType : int8_t
{
  JOINT_JOG = 0,
  TWIST = 1,
  POSE = 2,

  // Range of allowed values used for validation.
  MIN = JOINT_JOG,
  MAX = POSE
};

typedef std::pair<StatusCode, Eigen::VectorXd> JointDeltaResult;

// The joint jog command, this will be vector of length equal to the number of joints of the robot.
struct JointJogCommand
{
  std::vector<std::string> names;
  std::vector<double> velocities;
};

// The twist command,  frame_id is the name of the frame in which the command is specified in.
// frame_id must always be specified.
struct TwistCommand
{
  std::string frame_id;
//  Eigen::Vector<double, 6> velocities;
  Eigen::VectorXd velocities;
};

// The Pose command,  frame_id is the name of the frame in which the command is specified in.
// frame_id must always be specified.
struct PoseCommand
{
  std::string frame_id;
  Eigen::Isometry3d pose;
};

// The generic input type for servo that can be JointJog, Twist or Pose.
typedef std::variant<JointJogCommand, TwistCommand, PoseCommand> ServoInput;

// The output datatype of servo, this structure contains the names of the joints along with their positions, velocities and accelerations.
struct KinematicState
{
  std::vector<std::string> joint_names;
  Eigen::VectorXd positions, velocities, accelerations;
  ros::Time time_stamp;

  explicit KinematicState(const int num_joints)
  {
    joint_names.resize(num_joints);
    positions = Eigen::VectorXd::Zero(num_joints);
    velocities = Eigen::VectorXd::Zero(num_joints);
    accelerations = Eigen::VectorXd::Zero(num_joints);
  }

  KinematicState()
  = default;
};

// TODO: should add IK instance
class ManipulatorInstance
{
  ServoParameters servoParameters;
  KinematicState kinematicState;
  std::string IK_Plugins;
};
}  // namespace manipulator_servo
