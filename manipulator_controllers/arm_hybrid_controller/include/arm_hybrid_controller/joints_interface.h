//
// Created by lsy on 23-11-1.
//

#pragma once
#include <control_toolbox/pid.h>
#include <effort_controllers/joint_effort_controller.h>

namespace arm_hybrid_controller
{
class Joint
{
public:
    std::string name_;
    double exe_effort_;
    control_toolbox::Pid* position_pid_{};
    effort_controllers::JointEffortController* effort_ctrl_;
};
class JointsInterface
{
public:
    void init(hardware_interface::RobotHW *robot_hw,ros::NodeHandle &controller_nh)
    {
        const std::vector<std::string>& joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();
        ros::NodeHandle nh_position_pids(controller_nh, "gains");
        ros::NodeHandle nh_joints(controller_nh, "joints");
        hardware_interface::EffortJointInterface* effort_joint_interface;
        effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        num_hw_joints_ = static_cast<int>(joint_names.size());
        for (int i = 0; i < num_hw_joints_; i++)
            jnt_states_.push_back(robot_hw->get<hardware_interface::JointStateInterface>()->getHandle(joint_names[i]));
        for (const auto& name : joint_names)
        {
            ros::NodeHandle nh_joint(nh_joints, name);
            ros::NodeHandle nh_pid(nh_position_pids, name);
            Joint j{ .name_ = name,
                    .position_pid_ = new control_toolbox::Pid(),
                    .effort_ctrl_ = new effort_controllers::JointEffortController()};
            j.position_pid_->init(nh_pid);
            j.effort_ctrl_->init(effort_joint_interface, nh_joint);
            joints_.push_back(j);
        }
    }
    void moveJoint(const ros::Time &time, const ros::Duration &period)
    {
        for (int i = 0; i < num_hw_joints_; ++i) {
            joints_[i].effort_ctrl_->command_buffer_.writeFromNonRT(joints_[i].exe_effort_);
            joints_[i].effort_ctrl_->update(time, period);
        }
    }



    int num_hw_joints_;
    std::vector<Joint> joints_{};
    std::vector<std::string> joint_names_{};
    std::vector<hardware_interface::JointStateHandle> jnt_states_;
};
}
