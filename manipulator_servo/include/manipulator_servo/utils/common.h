//
// Created by lsy on 24-8-4.
//
// ref: https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo

#pragma once

#include <manipulator_servo/utils/datatypes.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <deque>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace manipulator_servo
{
// A minimum of 3 points are used to help with interpolation when creating trajectory messages.
constexpr int MIN_POINTS_FOR_TRAJ_MSG = 3;

/**
 * \brief Checks if a given VectorXd is a valid command.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const Eigen::VectorXd& command);

/**
 * \brief Checks if a given Isometry3d (pose) is a valid command.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const Eigen::Isometry3d& command);

/**
 * \brief Checks if a given Twist command is valid.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const TwistCommand& command);

/**
 * \brief Checks if a given Pose command is valid.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const PoseCommand& command);

/**
 * \brief Create a pose message for the provided change in Cartesian position.
 * @param delta_x The change in Cartesian position.
 * @param base_to_tip_frame_transform The transformation from robot base to ee frame.
 * @return The pose message.
 */
geometry_msgs::Pose poseFromCartesianDelta(const Eigen::VectorXd& delta_x,
                                                const Eigen::Isometry3d& base_to_tip_frame_transform);

/**
 * \brief Create a trajectory message from a rolling window queue of joint state commands. Method optionally returns a
 * trajectory message if one can be created.
 * @param servo_params The configuration used by servo, required for setting some field of the trajectory message.
 * @param joint_cmd_rolling_window A rolling window queue of joint state commands.
 * @return The trajectory message.
 */
std::optional<trajectory_msgs::JointTrajectory>
composeTrajectoryMessage(const ServoParameters& servo_params, const std::deque<KinematicState>& joint_cmd_rolling_window);

/**
 * \brief Adds a new joint state command to a queue containing commands over a time window. Also modifies the velocities
 * of the commands to help avoid overshooting.
 * @param next_joint_state The next commanded joint state.
 * @param joint_cmd_rolling_window Queue of containing a rolling window of joint commands.
 * @param max_expected_latency The next_joint_state will be added to the joint_cmd_rolling_window with a time stamp of
 * @param cur_time The current time stamp when the method is called. This value is used to update the time stamp of
 * next_joint_state
 */
void updateSlidingWindow(KinematicState& next_joint_state, std::deque<KinematicState>& joint_cmd_rolling_window,
                         double max_expected_latency, const ros::Time& cur_time);

/**
 * \brief Create a Float64MultiArray message from given joint state
 * @param servo_params The configuration used by servo, required for selecting position vs velocity.
 * @param joint_state The joint state to be added into the Float64MultiArray.
 * @return The Float64MultiArray message.
 */
std_msgs::Float64MultiArray composeMultiArrayMessage(const ServoParameters& servo_params,
                                                          const KinematicState& joint_state);

/**
 * \brief Computes scaling factor for velocity when the robot is near a singularity.
 * @param robot_state A pointer to the current robot state.
 * @param target_delta_x The vector containing the required change in Cartesian position.
 * @param servo_params The servo parameters, contains the singularity thresholds.
 * @return The velocity scaling factor and the reason for scaling.
 */

// TODO: should add IK instance
std::pair<double, StatusCode> velocityScalingFactorForSingularity(const std::string IK_plugin,
                                                                  const Eigen::VectorXd& target_delta_x,
                                                                  const ServoParameters& servo_params);

/**
 * \brief Apply velocity scaling based on joint limits. If the robot model does not have velocity limits defined,
 * then a scale factor of 1.0 will be returned.
 * @param velocities The commanded velocities.
 * @param joint_bounds The bounding information for the robot joints.
 * @param scaling_override The user defined velocity scaling override.
 * @return The velocity scaling factor.
 */

// TODO: should add Joint bounds
double jointLimitVelocityScalingFactor(const Eigen::VectorXd& velocities,
                                       const std::string joint_bounds, double scaling_override);

/**
 * \brief Finds the joints that are exceeding allowable position limits.
 * @param positions The joint positions.
 * @param velocities The current commanded velocities.
 * @param joint_bounds The allowable limits for the robot joints.
 * @param margins Additional buffer on the actual joint limits.
 * @return The joints that are violating the specified position limits.
 */

// TODO: should add Joint bounds
std::vector<int> jointsToHalt(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                              const std::string joint_bounds, const std::vector<double>& margins);

/**
 * \brief Helper function for converting Eigen::Isometry3d to geometry_msgs/TransformStamped.
 * @param eigen_tf The isometry to be converted to TransformStamped.
 * @param parent_frame The target frame.
 * @param child_frame The current frame.
 * @return The isometry as a TransformStamped message.
 */
geometry_msgs::TransformStamped convertIsometryToTransform(const Eigen::Isometry3d& eigen_tf,
                                                                const std::string& parent_frame,
                                                                const std::string& child_frame);

/**
 * \brief Convert a PoseStamped message to a Servo Pose
 * @param msg The PoseStamped message.
 * @return The equivalent Servo Pose type.
 */
PoseCommand poseFromPoseStamped(const geometry_msgs::PoseStamped& msg);


}  // namespace manipulator_servo


