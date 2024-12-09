//
// Created by lsy on 24-8-4.
//
// ref: https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo

#pragma once

#include <manipulator_servo/utils/common.h>
#include <tf2_eigen/tf2_eigen.h>

namespace manipulator_servo
{

/**
 * \brief Compute the change in joint position for the given joint jog command.
 * @param command The joint jog command.
 * @param robot_state_ The current robot state as obtained from PlanningSceneMonitor.
 * @param servo_params The servo parameters.
 * @param joint_name_group_index_map Mapping between joint subgroup name and move group joint vector position.
 * @return The status and joint position change required (delta).
 */
JointDeltaResult jointDeltaFromJointJog(const JointJogCommand& command, const ManipulatorInstance& manipulatorInstance);

/**
 * \brief Compute the change in joint position for the given twist command.
 * @param command The twist command.
 * @param robot_state_ The current robot state as obtained from PlanningSceneMonitor.
 * @param servo_params The servo parameters.
 * @param planning_frame The planning frame name.
 * @param joint_name_group_index_map Mapping between joint subgroup name and move group joint vector position.
 * @return The status and joint position change required (delta).
 */
JointDeltaResult jointDeltaFromTwist(const TwistCommand& command, const ManipulatorInstance& manipulatorInstance, const std::string& planning_frame);

/**
 * \brief Compute the change in joint position for the given pose command.
 * @param command The pose command.
 * @param robot_state_ The current robot state as obtained from PlanningSceneMonitor.
 * @param servo_params The servo parameters.
 * @param planning_frame The planning frame name.
 * @param ee_frame The end effector frame name.
 * @param joint_name_group_index_map Mapping between sub group joint name and move group joint vector position
 * @return The status and joint position change required (delta).
 */
JointDeltaResult jointDeltaFromPose(const PoseCommand& command, const ManipulatorInstance& manipulatorInstance, const std::string& planning_frame,
                                    const std::string& ee_frame);

/**
 * \brief Computes the required change in joint angles for given Cartesian change, using the robot's IK solver.
 * @param cartesian_position_delta The change in Cartesian position.
 * @param robot_state_ The current robot state as obtained from PlanningSceneMonitor.
 * @param servo_params The servo parameters.
 * @param joint_name_group_index_map Mapping between joint subgroup name and move group joint vector position.
 * @return The status and joint position change required (delta).
 */
JointDeltaResult jointDeltaFromIK(const Eigen::VectorXd& cartesian_position_delta,
                                  const ManipulatorInstance& manipulatorInstance);

}  // namespace manipulator_servo
