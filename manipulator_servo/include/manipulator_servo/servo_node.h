//
// Created by lsy on 24-8-4.
//

#pragma once

#include <control_msgs/JointJog.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <manipulator_servo/servo.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include <trajectory_msgs/JointTrajectory.h>

// #include <control_msgs//joint_jog.hpp>
//#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <geometry_msgs/msg/twist_stamped.hpp>
//#include <moveit_msgs/srv/servo_command_type.hpp>
//#include <moveit_msgs/msg/servo_status.hpp>

namespace manipulator_servo
{

class ServoNode
{
public:
  explicit ServoNode(const ros::NodeHandle nh);

  ~ServoNode();

  // Disable copy construction.
  ServoNode(const ServoNode&) = delete;

  // Disable copy assignment.
  ServoNode& operator=(ServoNode&) = delete;


private:
  /**
   * \brief Loop that handles different types of incoming commands.
   */
  void servoLoop();

  /**
   * \brief The service to pause servoing, this does not exit the loop or stop the servo loop thread.
   * The loop will be alive even after pausing, but no commands will be processed.
   */
  void pauseServo(const std::shared_ptr<std_srvs::SetBool::Request>& request,
                  const std::shared_ptr<std_srvs::SetBool::Response>& response);

  /**
   * \brief The service to set the command type for Servo.
   * Supported command types can be found in utils/datatypes.hpp
   * This service must be used to set the command type before sending any servoing commands.
   */
//  void switchCommandType(const std::shared_ptr<moveit_msgs::ServoCommandType::Request>& request,
//                         const std::shared_ptr<moveit_msgs::ServoCommandType::Response>& response);

  void jointJogCallback(const control_msgs::JointJog::ConstPtr& msg);
  void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  std::optional<KinematicState> processJointJogCommand(const manipulator_servo::ManipulatorInstance);
  std::optional<KinematicState> processTwistCommand(const manipulator_servo::ManipulatorInstance);
  std::optional<KinematicState> processPoseCommand(const manipulator_servo::ManipulatorInstance);

  // Variables

  const ros::NodeHandle nh_;
  std::unique_ptr<Servo> servo_;
//  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  KinematicState last_commanded_state_;  // Used when commands go stale;
  control_msgs::JointJog latest_joint_jog_;
  geometry_msgs::TwistStamped latest_twist_;
  geometry_msgs::PoseStamped latest_pose_;

//  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;
//  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
//  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
//
//  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr multi_array_publisher_;
//  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
//  rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr status_publisher_;

  ros::Subscriber joint_jog_subscriber_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber pose_subscriber_;

  ros::Publisher multi_array_publisher_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher servo_status_publisher_;

//  ros::Service<moveit_msgs::ServoCommandType>::SharedPtr switch_command_type_;
//  ros::Service<std_srvs::SetBool>::SharedPtr pause_servo_;

  ros::ServiceServer pause_servo_service_;

  // Used for communication with thread
  std::atomic<bool> stop_servo_;
  std::atomic<bool> servo_paused_;
  std::atomic<bool> new_joint_jog_msg_, new_twist_msg_, new_pose_msg_;

  // Threads used by ServoNode
  std::thread servo_loop_thread_;

  // rolling window of joint commands
  std::deque<KinematicState> joint_cmd_rolling_window_;
};

} //namespace manipulator_servo

