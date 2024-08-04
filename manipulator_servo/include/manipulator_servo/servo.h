//
// Created by lsy on 24-8-4.
//

#pragma once

//#include <manipulator_servo/collision_monitor.hpp>
#include <manipulator_servo/utils/command.h>
#include <manipulator_servo/utils/datatypes.h>
//#include <moveit/kinematics_base/kinematics_base.h>
//#include <moveit/online_signal_smoothing/smoothing_base_class.h>
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <variant>
#include <queue>

namespace manipulator_servo
{

class Servo
{
public:
  Servo(const ros::NodeHandle nh);

  ~Servo();

  // Disable copy construction.
  Servo(const Servo&) = delete;

  // Disable copy assignment.
  Servo& operator=(Servo&) = delete;

  /**
   * \brief Computes the joint state required to follow the given command.
   * @param robot_state RobotStatePtr instance used for calculating the next joint state.
   * @param command The command to follow, std::variant type, can handle JointJog, Twist and Pose.
   * @return The required joint state.
   */
  KinematicState getNextJointState(const ManipulatorInstance& manipulatorInstance, const ServoInput& command);

  /**
   * \brief Set the type of incoming servo command.
   * @param command_type The type of command servo should expect.
   */
  void setCommandType(const CommandType& command_type);

  /**
   * \brief Get the type of command that servo is currently expecting.
   * @return The type of command.
   */
  CommandType getCommandType() const;

  /**
   * \brief Get the current status of servo.
   * @return The current status.
   */
  StatusCode getStatus() const;

  /**
   * \brief Get the message associated with the current servo status.
   * @return The status message.
   */
  std::string getStatusMessage() const;

  /**
   * \brief Start/Stop collision checking thread.
   * @param check_collision Stops collision checking thread if false, starts it if true.
   */
  void setCollisionChecking(const bool check_collision);

  /**
   * \brief Get the current state of the robot as given by planning scene monitor.
   * @return The current state of the robot.
   */
  KinematicState getCurrentRobotState() const;

  /**
   * \brief Smoothly halt at a commanded state when command goes stale.
   * @param halt_state The desired stop state.
   * @return The next state stepping towards the required halting state.
   */
  std::pair<bool, KinematicState> smoothHalt(const KinematicState& halt_state);

  /**
   * \brief Applies smoothing to an input state, if a smoothing plugin is set.
   * @param state The state to be updated by the smoothing plugin.
   */
  void doSmoothing(KinematicState& state);

  /**
   * \brief Resets the smoothing plugin, if set, to a specified state.
   * @param state The desired state to reset the smoothing plugin to.
   */
  void resetSmoothing(const KinematicState& state);

private:
  /**
   * \brief Finds the transform from the planning frame to a specified command frame.
   * If the command frame is part of the robot model, directly look up the transform using the robot model.
   * Else, fall back to using TF to look up the transform.
   * @param command_frame The command frame name.
   * @param planning_frame The planning frame name.
   * @return The transformation between planning frame and command frame, or std::nullopt if there was a failure looking
   * up a transform.
   */
  std::optional<Eigen::Isometry3d> getPlanningToCommandFrameTransform(const std::string& command_frame,
                                                                      const std::string& planning_frame) const;

  /**
   * \brief Convert a given twist command to planning frame,
   * The command frame specified by `command.frame_id` is expected to be a stationary frame or end-effector frame.
   * References:
   * https://core.ac.uk/download/pdf/154240607.pdf
   * https://www.seas.upenn.edu/~meam520/notes02/Forces8.pdf
   * @param command The twist command to be converted.
   * @param planning_frame The name of the planning frame.
   * @return The transformed twist command.
   */
  std::optional<TwistCommand> toPlanningFrame(const TwistCommand& command, const std::string& planning_frame) const;

  /**
   * \brief Convert a given pose command to planning frame
   * @param command The pose command to be converted.
   * @param planning_frame The name of the planning frame.
   * @return The transformed pose command.
   */
  std::optional<PoseCommand> toPlanningFrame(const PoseCommand& command, const std::string& planning_frame) const;

  /**
   * \brief Compute the change in joint position required to follow the received command.
   * @param command The incoming servo command.
   * @param robot_state RobotStatePtr instance used for calculating the command.
   * @return The joint position change required (delta).
   */
  Eigen::VectorXd jointDeltaFromCommand(const ServoInput& command, const ManipulatorInstance& manipulatorInstance);

  /**
   * \brief Validate the servo parameters
   * @param servo_params The servo parameters
   * @return True if parameters are valid, else False
   */
  bool validateParams(const ServoParameters& servo_params) const;

  /**
   * \brief Updates the servo parameters and performs validations.
   */
  bool updateParams();

  /**
   * \brief Create and initialize the smoothing plugin to be used by servo.
   */
  void setSmoothingPlugin();

  /**
   * \brief Apply halting logic to specified joints.
   * @param joints_to_halt The indices of joints to be halted.
   * @param current_state The current kinematic state.
   * @param target_state The target kinematic state.
   * @return The bounded kinematic state.
   */
  KinematicState haltJoints(const std::vector<int>& joints_to_halt, const KinematicState& current_state,
                            const KinematicState& target_state) const;

  // Variables

  StatusCode servo_status_;
  // This needs to be threadsafe so it can be updated in realtime.
  std::atomic<CommandType> expected_command_type_;

  ManipulatorInstance manipulatorInstance_;

//  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // This value will be updated by CollisionMonitor in a separate thread.
  std::atomic<double> collision_velocity_scale_ = 1.0;
//  std::unique_ptr<CollisionMonitor> collision_monitor_;

  // Pointer to the (optional) smoothing plugin.
//  pluginlib::UniquePtr<online_signal_smoothing::SmoothingBaseClass> smoother_ = nullptr;

};

}  // namespace manipulator_servo
