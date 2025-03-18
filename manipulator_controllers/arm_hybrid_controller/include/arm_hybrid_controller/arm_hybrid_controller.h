//
// Created by lsy on 23-10-25.
//

#pragma once

#include <functional>
#include <boost/dynamic_bitset.hpp>

// dynamics
#include <dynamics_interface/dynamics_interface.h>

// controller_interface
#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <effort_controllers/joint_effort_controller.h>

// messages
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <arm_msgs/ChangeHybridMode.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// actionlib
#include <actionlib/server/action_server.h>

// some custom interface
#include <arm_common/tools/lp_filter.h>
#include <arm_hybrid_controller/controller_state_interface.h>
#include <arm_hybrid_controller/joints_interface.h>

// joint_trajectory_controller_interface
#include <joint_trajectory_interface/init_joint_trajectory.h>
#include <joint_trajectory_interface/creat_joint_trajectory.h>
#include <joint_trajectory_interface/joint_trajectory_segment.h>
#include <joint_trajectory_interface/init_joint_trajectory.h>
#include <joint_trajectory_interface/hold_trajectory_builder.h>
#include <joint_trajectory_interface/stop_trajectory_builder.h>

// trajectory_interface
#include <trajectory_interface/quintic_spline_segment.h>

// realtime_tools
#include <realtime_tools/realtime_server_goal_handle.h>
namespace arm_hybrid_controller
{
namespace internal {
    template<class Enclosure, class Member>
    inline boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member) {
        actionlib::EnclosureDeleter<Enclosure> d(enclosure);
        boost::shared_ptr<Member> p(&member, d);
        return p;
    }
}
class ArmHybridController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
        hardware_interface::JointStateInterface>
{
public:
    ArmHybridController() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& /*time*/) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

private:
    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
    typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
    typedef ActionServer::GoalHandle                                                            GoalHandle;
    typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
    typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
    typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;
    typedef realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>     StatePublisher;
    typedef std::unique_ptr<StatePublisher>                                                     StatePublisherPtr;

    typedef joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::QuinticSplineSegment<double>>          Segment;
    typedef std::vector<Segment>                                                                TrajectoryPerJoint;
    typedef std::vector<TrajectoryPerJoint>                                                     Trajectory;
    typedef std::shared_ptr<Trajectory>                                                         TrajectoryPtr;
    typedef std::shared_ptr<TrajectoryPerJoint>                                                 TrajectoryPerJointPtr;
    typedef realtime_tools::RealtimeBox<TrajectoryPtr>                                          TrajectoryBox;
    typedef typename Segment::Scalar                                                            Scalar;

    void moveJoint(const ros::Time& time, const ros::Duration& period);
    void commandCB(const geometry_msgs::PointStampedConstPtr & msg)
    {
//        cmd_rt_buffer_.writeFromNonRT(*msg);
    }
    bool changeHybridMode(arm_msgs::ChangeHybridModeRequest &req,arm_msgs::ChangeHybridModeResponse &res);
    void changeMode(int mode);
    void gravity_compensation();
    void trajectory_teaching(const ros::Time& now);
    void trajectory_tracking(const ros::Time& now,const ros::Duration& period);
    void holding_position(const ros::Time& now);
    bool updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string);
    void updateFuncExtensionPoint(const Trajectory& curr_traj, const TimeData& time_data)
    {
        // To be implemented by derived class
    }

    // Action function
    void goalCB(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
    void cancelCB(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
    void preemptActiveGoal();
    enum
    {
        GRAVITY_COMPENSATION,
        TRAJECTORY_TEACHING,
        TRAJECTORY_TRACKING,
        HOLDING_POSITION
    };

    int mode_;
    std::string name_;
    ros::Time last_time_;
    ros::Timer goal_handle_timer_;
    ros::Time new_gl_time_;
    ros::Time trajectory_points_time_;

    ros::ServiceServer change_mode_server_;
    ros::Duration action_monitor_period_;
    ros::NodeHandle    controller_nh_;
    std::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>> action_server_;

//   use for trajectory_teaching
    bool record_teach_time_ = false;
    int teach_points_num_ = 0;
    double teach_sample_interval_;
    ros::Time start_teach_time_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> teach_points_{};
    trajectory_msgs::JointTrajectoryPtr teach_trajectory_{};
//   int point_current_=0;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points_;
    dynamics_interface::DynamicsInterface dynamics_interface_;
    JointsInterface joints_interface_;
    ControllerStateInterface controller_state_interface_{};
    std::vector<std::string> joint_names_;
    int num_hw_joints_;
    joint_trajectory_controller::SegmentTolerances<double> default_tolerances_; ///< Default trajectory segment tolerances.

    RealtimeGoalHandlePtr     rt_active_goal_;     ///< Currently active action goal, if any.
    TrajectoryBox curr_trajectory_box_;
    boost::dynamic_bitset<> successful_joint_traj_;
};
}// namespace arm_hybrid_controller

PLUGINLIB_EXPORT_CLASS(arm_hybrid_controller::ArmHybridController, controller_interface::ControllerBase)