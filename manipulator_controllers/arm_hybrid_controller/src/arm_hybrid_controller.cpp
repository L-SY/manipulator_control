//
// Created by lsy on 23-10-25.
//

#include "arm_hybrid_controller/arm_hybrid_controller.h"
namespace arm_hybrid_controller
{
bool ArmHybridController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
{
    // Action status checking update rate
    double action_monitor_rate = 20.0;
    controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
    action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
    ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");
    name_ = controller_nh.getNamespace();
    controller_nh_ = controller_nh;
    const std::vector<std::string>& joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();
    joint_names_ = joint_names;
    num_hw_joints_ = static_cast<int>(joint_names_.size());
    change_mode_server_ = controller_nh_.advertiseService("change_hybrid_mode", &ArmHybridController::changeHybridMode, this);
    // Init controller_state_publisher
    controller_state_interface_.init(controller_nh_,joint_names_);
    joints_interface_.init(robot_hw,controller_nh_);
    ros::NodeHandle nh_dynamics(controller_nh_, "dynamics");
    dynamics_interface_.init(nh_dynamics);

    // Default tolerances
    ros::NodeHandle tol_nh(controller_nh_, "constraints");
    default_tolerances_ = joint_trajectory_controller::getSegmentTolerances<Scalar>(tol_nh, joint_names_);

    // For teaching
    ros::NodeHandle teaching_nh(controller_nh_, "teaching");
    teaching_nh.getParam("teach_sample_interval",teach_sample_interval_);
    teach_trajectory_ = boost::make_shared<trajectory_msgs::JointTrajectory>();
    // Action Service
    action_server_.reset(
            new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(controller_nh_, "follow_joint_trajectory",
                             std::bind(&ArmHybridController::goalCB, this, std::placeholders::_1),
                             std::bind(&ArmHybridController::cancelCB, this, std::placeholders::_1), false));
    action_server_->start();

    last_time_ = ros::Time::now();
    return true;
}

void ArmHybridController::changeMode(int mode)
{
    mode_ = mode;
    std::string mode_string;
    switch (mode_)
    {
        case GRAVITY_COMPENSATION:
            mode_string = "GRAVITY_COMPENSATION";
            break;
        case TRAJECTORY_TEACHING:
            mode_string = "TRAJECTORY_TEACHING";
            break;
        case TRAJECTORY_TRACKING:
            mode_string = "TRAJECTORY_TRACKING";
            break;
        case HOLDING_POSITION:
        {
            mode_string = "HOLDING_POSITION";
            controller_state_interface_.initDesiredState(joints_interface_.jnt_states_);
        }
            break;
    }
    std::string reply = "Now the mode is:"+ mode_string;
    ROS_INFO_STREAM(reply);
}
bool ArmHybridController::changeHybridMode(arm_msgs::ChangeHybridModeRequest &req,arm_msgs::ChangeHybridModeResponse &res)
{
    int mode = static_cast<int>(req.new_mode);
    changeMode(mode);
    return true;
}
void ArmHybridController::starting(const ros::Time& time)
{
    controller_state_interface_.initTimeData(time);
//    mode_ = GRAVITY_COMPENSATION;
    mode_ = HOLDING_POSITION;
    controller_state_interface_.initDesiredState(joints_interface_.jnt_states_);
}

inline void ArmHybridController::stopping(const ros::Time& /*time*/)
{
    preemptActiveGoal();
}

void ArmHybridController::moveJoint(const ros::Time &time, const ros::Duration &period)
{
    joints_interface_.moveJoint(time,period);
}
void ArmHybridController::update(const ros::Time &time, const ros::Duration &period)
{
    switch (mode_)
    {
        case GRAVITY_COMPENSATION:
            gravity_compensation();
            break;
        case TRAJECTORY_TEACHING:
            trajectory_teaching(time);
            break;
        case TRAJECTORY_TRACKING:
            trajectory_tracking(time,period);
            break;
        case HOLDING_POSITION:
            holding_position(time);
            break;
    }
    if(mode_ != TRAJECTORY_TRACKING)
        controller_state_interface_.updateTimeData(time,period);
    dynamics_interface_.pubDynamics();
    controller_state_interface_.update(time,joints_interface_.jnt_states_);
    last_time_ = time;
    if(dynamics_interface_.send_tau_)
        moveJoint(time,period);
}
void ArmHybridController::gravity_compensation()
{
    dynamics_interface_.computerInverseDynamics(joints_interface_.jnt_states_);
    for (int i = 0; i < joints_interface_.num_hw_joints_; ++i) {
        joints_interface_.joints_[i].exe_effort_ = dynamics_interface_.tau_without_a_(i);
    }
    if (teach_points_num_)
    {
        std::string error_string;
        teach_trajectory_->header.stamp = ros::Time::now();
        teach_trajectory_->points = teach_points_;
        teach_trajectory_->joint_names = joint_names_;
        const bool update_ok = updateTrajectoryCommand(teach_trajectory_,rt_active_goal_,&error_string);
        if (update_ok)
        {
            record_teach_time_ = false;
            teach_points_num_ = 0;
            teach_points_.clear();
        }
    }
}
void ArmHybridController::trajectory_teaching(const ros::Time& now)
{
    if (!record_teach_time_)
    {
        record_teach_time_ = true;
        start_teach_time_ = now;
    }
    dynamics_interface_.computerInverseDynamics(joints_interface_.jnt_states_);
    for (int i = 0; i < joints_interface_.num_hw_joints_; ++i) {
        joints_interface_.joints_[i].exe_effort_ = dynamics_interface_.tau_without_a_(i);
    }
    if ( (now - start_teach_time_).toSec() > (teach_points_num_+1)*teach_sample_interval_)
    {
        ROS_INFO_STREAM("TIME"<<(now - start_teach_time_).toSec());
        trajectory_msgs::JointTrajectoryPoint point;
        for (int i = 0; i < num_hw_joints_; ++i) {
            point.positions.push_back(controller_state_interface_.current_state_.position[i]);
            point.velocities.push_back(controller_state_interface_.current_state_.velocity[i]);
            point.accelerations.push_back(controller_state_interface_.current_state_.acceleration[i]);
            point.time_from_start = now - start_teach_time_;
        }
        teach_points_.push_back(point);
        teach_points_num_++;
    }
}
void ArmHybridController::holding_position(const ros::Time& now)
{
    for (int i = 0; i < joints_interface_.num_hw_joints_; ++i) {
        double position_pid_value = joints_interface_.joints_[i].position_pid_->computeCommand(controller_state_interface_.state_error_.position[i],now-last_time_);
        double cmd = position_pid_value;
        joints_interface_.joints_[i].exe_effort_ = cmd;
    }
}
void ArmHybridController::trajectory_tracking(const ros::Time& time, const ros::Duration& period) {
    // Get currently followed trajectory
    TrajectoryPtr curr_traj_ptr;
    curr_trajectory_box_.get(curr_traj_ptr);
    if (curr_traj_ptr == nullptr)
    {
        ROS_INFO_STREAM("NO msg");
        changeMode(HOLDING_POSITION);
    }
    // Curr_traj is use for computer tolerance.
    else
    {
//        Trajectory& curr_traj = *curr_traj_ptr;
        controller_state_interface_.old_time_data_ = *(controller_state_interface_.time_data_.readFromRT());
        TimeData time_data;
        time_data.time   = time;                                     // Cache current time
        time_data.period = period;                                   // Cache current control period
        time_data.uptime = controller_state_interface_.old_time_data_.uptime + period; // Update controller uptime
        controller_state_interface_.time_data_.writeFromNonRT(time_data);
//        controller_state_interface_.updateTimeData(time,period);
//        controller_state_interface_.updateDesiredStates<Trajectory>(trajectory_points_time_, curr_traj_ptr.get());
        // Update current state and state error
        controller_state_interface_.updateDesiredStates<Trajectory>(time_data.uptime, curr_traj_ptr.get());

//        changeMode(HOLDING_POSITION);

/*      For error check
        for (int i = 0; i < num_hw_joints_; ++i)
        {
            typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], time_data.uptime.toSec(), controller_state_interface_.desired_joint_state_);
            if (curr_traj[i].end() == segment_it)
            {
                // Non-realtime safe, but should never happen under normal operation
                ROS_ERROR_NAMED(name_,"Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
                changeMode(HOLDING_POSITION);
                return;
            }
            // Get state error for current joint
            controller_state_interface_.state_joint_error_.position[0] = controller_state_interface_.state_error_.position[i];
            controller_state_interface_.state_joint_error_.velocity[0] = controller_state_interface_.state_error_.velocity[i];
            controller_state_interface_.state_joint_error_.acceleration[0] = controller_state_interface_.state_error_.acceleration[i];

            //Check tolerances
            const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
            if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
            {
                // Check tolerances
                if (controller_state_interface_.time_data_.readFromRT()->uptime.toSec() < segment_it->endTime())
                {
                    // Currently executing a segment: check path tolerances
                    const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& joint_tolerances = segment_it->getTolerances();
                    if (!checkStateTolerancePerJoint(controller_state_interface_.state_joint_error_, joint_tolerances.state_tolerance))
                    {
                        rt_segment_goal->preallocated_result_->error_code =
                                control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
                        rt_segment_goal->preallocated_result_->error_string = joint_names_[i] + " path error " + std::to_string( controller_state_interface_.state_joint_error_.position[0] );
                        rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
                        // Force this to run before destroying rt_active_goal_ so results message is returned
                        rt_active_goal_->runNonRealtime(ros::TimerEvent());
                        rt_active_goal_.reset();
                        successful_joint_traj_.reset();
                    }
                }
                else if (segment_it == --curr_traj[i].end())
                {
                    // Controller uptime
                    const ros::Time uptime = controller_state_interface_.time_data_.readFromRT()->uptime;

                    // Checks that we have ended inside the goal tolerances
                    const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& tolerances = segment_it->getTolerances();
                    const bool inside_goal_tolerances = checkStateTolerancePerJoint(controller_state_interface_.state_joint_error_, tolerances.goal_state_tolerance);

                    if (inside_goal_tolerances)
                    {
                        successful_joint_traj_[i] = 1;
                    }
                    else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
                    {
                        // Still have some time left to meet the goal state tolerances
                    }
                    else
                    {
                        rt_segment_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
                        rt_segment_goal->preallocated_result_->error_string = joint_names_[i] + " goal error " + std::to_string(controller_state_interface_.state_joint_error_.position[0] );
                        rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
                        // Force this to run before destroying rt_active_goal_ so results message is returned
                        rt_active_goal_->runNonRealtime(ros::TimerEvent());
                        rt_active_goal_.reset();
                        successful_joint_traj_.reset();
                    }
                }
            }
        }

        //If there is an active goal and all segments finished successfully then set goal as succeeded
        RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
        if (current_active_goal && static_cast<int>(successful_joint_traj_.count()) == num_hw_joints_)
        {
            current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
            current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
            current_active_goal.reset(); // do not publish feedback
            rt_active_goal_.reset();
            successful_joint_traj_.reset();
        }
*/
        for (int i = 0; i < joints_interface_.num_hw_joints_; ++i) {
            double position_pid_value = joints_interface_.joints_[i].position_pid_->computeCommand(controller_state_interface_.state_error_.position[i],time-last_time_);
            double cmd = position_pid_value;
            joints_interface_.joints_[i].exe_effort_ = cmd;
        }
//        trajectory_points_time_+=period;
    }
}
void ArmHybridController::preemptActiveGoal()
{
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

    // Cancels the currently active goal
    if (current_active_goal)
    {
        // Marks the current goal as canceled
        rt_active_goal_.reset();
        current_active_goal->gh_.setCanceled();
    }
}

void ArmHybridController::cancelCB(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

    // Check that cancel request refers to currently active goal (if any)
    if (current_active_goal && current_active_goal->gh_ == gh)
    {
        // Reset current goal
        rt_active_goal_.reset();

        // Controller uptime
//        const ros::Time uptime = controller_state_interface_.time_data_.readFromRT()->uptime;

        // Enter hold current position mode
//        setHoldPosition(uptime);
        mode_ = HOLDING_POSITION;
        ROS_DEBUG_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");

        // Mark the current goal as canceled
        current_active_goal->gh_.setCanceled();
    }
}
void ArmHybridController::goalCB(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
    ROS_INFO_STREAM("goalCB");

    ROS_DEBUG_STREAM_NAMED(name_,"Received new action goal");
    // Precondition: Running controller
    if (!this->isRunning())
    {
        ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        gh.setRejected(result);
        return;
    }

    RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
    std::string error_string;
    const bool update_ok = updateTrajectoryCommand(internal::share_member(gh.getGoal(), gh.getGoal()->trajectory),rt_goal,&error_string);
    rt_goal->preallocated_feedback_->joint_names = joints_interface_.joint_names_;
    if (update_ok)
    {
        // Accept new goal
        preemptActiveGoal();
        gh.setAccepted();
        rt_active_goal_ = rt_goal;
        // Setup goal status checking timer
        goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,&RealtimeGoalHandle::runNonRealtime,rt_goal);
        goal_handle_timer_.start();
    }
    else
    {
        // Reject invalid goal
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        result.error_string = error_string;
        gh.setRejected(result);
    }
}

bool ArmHybridController::updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
    // Time data
    TimeData* time_data = controller_state_interface_.time_data_.readFromRT();
    // Time of the next update
    const ros::Time next_update_time = time_data->time + time_data->period;
    // Uptime of the next update
    ros::Time next_update_uptime = time_data->uptime + time_data->period;

    typedef joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> Options;
    Options options;
    TrajectoryPtr curr_traj_ptr;
    curr_trajectory_box_.get(curr_traj_ptr);

    options.other_time_base = &next_update_uptime;
    options.current_trajectory  = curr_traj_ptr.get();
    options.joint_names = &joint_names_;
    options.error_string = error_string;
    options.rt_goal_handle = gh;
    options.default_tolerances = &default_tolerances_;

    // Hold current position if trajectory is empty
    if (msg->points.empty())
    {
        changeMode(HOLDING_POSITION);
        ROS_INFO_STREAM("MSG IS EMPTY");
        ROS_DEBUG_NAMED(name_, "Empty trajectory command, stopping.");
        return true;
    }

    // Update currently executing trajectory
    std::string err_string;
    try
    {
        TrajectoryPtr traj_ptr(new Trajectory);
        *traj_ptr = joint_trajectory_controller::initJointTrajectory<Trajectory>(*msg, next_update_time,options);
        if (!traj_ptr->empty())
        {
            curr_trajectory_box_.set(traj_ptr);
            changeMode(TRAJECTORY_TRACKING);
//            trajectory_points_time_ = msg.get()->header.stamp.now();
            ROS_INFO_STREAM("CREAT WHOLE TRAJECTORY");
        }
        else
        {
            return false;
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(name_, ex.what());
        return false;
    }
    return true;
}
}// namespace arm_hybrid_controller