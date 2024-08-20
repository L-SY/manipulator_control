//
// Created by lsy on 24-5-23.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <effort_controllers/joint_effort_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>

#include "std_msgs/Float64.h"
#include "manipulator_msgs/GripperState.h"
#include "manipulator_msgs/GripperCmd.h"

namespace gripper_controller {
class gripperController : public controller_interface::MultiInterfaceController<
                              hardware_interface::EffortJointInterface> {
  enum ControllerState { NORMAL, SHUTDOWN };

public:
  gripperController();
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;
  void starting(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;

  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  void getGains(double &p, double &i, double &d, double &i_max, double &i_min,
                bool &antiwindup);

  void setGains(const double &p, const double &i, const double &d,
                const double &i_max, const double &i_min,
                const bool &antiwindup = false);

private:
  void normal(const ros::Time &time, const ros::Duration &period);
  void shutdown(const ros::Time &time, const ros::Duration &period);
  void commandCB(const manipulator_msgs::GripperCmd& msg);
  void enforceJointLimits(double &command);

  int loopCount_;
  control_toolbox::Pid pidController_;

  std::unique_ptr<realtime_tools::RealtimePublisher<manipulator_msgs::GripperState>> statePub_;
  int controllerState_ = NORMAL;
  ros::Time startTime_;
  bool stateChanged_ = false;
  ros::Subscriber cmdSub_;
  hardware_interface::JointHandle jointHandle_;
  urdf::JointConstSharedPtr jointUrdf_;
  realtime_tools::RealtimeBuffer<manipulator_msgs::GripperCmd> cmdRtBuffer_{};
};

} // namespace gripper_controller