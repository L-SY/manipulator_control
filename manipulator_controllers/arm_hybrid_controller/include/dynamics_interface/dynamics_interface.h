//
// Created by lsy on 23-10-30.
//

#pragma once

#include <pinocchio/fwd.hpp>
//must add fwd.hpp before other .h for solve the conflict between ROS and pinocchio
//https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/dynamics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <hardware_interface/joint_state_interface.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <arm_common/tools/lp_filter.h>
#include <arm_common/tools/ros_param.h>

namespace dynamics_interface
{
class DynamicsInterface
{
public:
    bool init(ros::NodeHandle &controller_nh);
    void pubDynamics();
    void computerInverseDynamics(std::vector<hardware_interface::JointStateHandle> jnt_states);
    Eigen::VectorXd tau_{},tau_without_a_{},tau_without_a_v_{};
    Eigen::VectorXd q_{},v_{},last_v_{},a_{},zero_{};
    bool send_tau_ = false;

private:
    pinocchio::Model model_;
    std::string urdf_filename_;
    pinocchio::Data pinocchio_data_;

    ros::Time last_time_{};
    ros::NodeHandle node_;
    ros::Publisher error_pub_,tau_pub_,tau_without_a_pub_,tau_without_a_v_pub_,tau_exe_pub_,a_pub_;
    std_msgs::Float64MultiArray tau_error_msg_{},tau_msg_{},tau_without_a_msg_{},tau_without_a_v_msg_{},tau_exe_msg_{},a_msg_{};

    int num_hw_joints_;
    LowPassFilter* a_lp_filter_;
};

bool DynamicsInterface::init(ros::NodeHandle &dynamics_nh)
{
    if (!dynamics_nh.hasParam("urdf_filename"))
    {
        ROS_ERROR_STREAM("NO URDF FILE PATH");
        return false;
    }
    else
    {
        dynamics_nh.getParam("urdf_filename",urdf_filename_);
        pinocchio::urdf::buildModel(urdf_filename_,model_);
        pinocchio_data_ = pinocchio::Data(model_);
    }
    if (!dynamics_nh.hasParam("send_tau"))
        send_tau_ = true;
    else
        dynamics_nh.getParam("send_tau",send_tau_);
    if (!dynamics_nh.hasParam("num_hw_joints"))
    {
        ROS_ERROR_STREAM("NO NUM_HW_JOINTS");
        return false;
    }
    else
        dynamics_nh.getParam("num_hw_joints",num_hw_joints_);
//  Init the dynamics relative var.
    zero_.resize(num_hw_joints_);
    tau_.resize(num_hw_joints_);
    tau_without_a_.resize(num_hw_joints_);
    tau_without_a_v_.resize(num_hw_joints_);
    q_.resize(num_hw_joints_);
    v_.resize(num_hw_joints_);
    last_v_.resize(num_hw_joints_);
    a_.resize(num_hw_joints_);
    a_lp_filter_ = new LowPassFilter(dynamics_nh);

//  Init the publisher relative var.
    tau_error_msg_.data.resize(num_hw_joints_);
    tau_msg_.data.resize(num_hw_joints_);
    tau_exe_msg_.data.resize(num_hw_joints_);
    tau_without_a_msg_.data.resize(num_hw_joints_);
    tau_without_a_v_msg_.data.resize(num_hw_joints_);
    a_msg_.data.resize(num_hw_joints_);
    a_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("a", 1);
    error_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_error", 1);
    tau_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau", 1);
    tau_exe_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_exe", 1);
    tau_without_a_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_without_a", 1);
    tau_without_a_v_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_without_a_v", 1);
    return true;
}
void DynamicsInterface::pubDynamics()
{
//  Pub all data
    error_pub_.publish(tau_error_msg_);
    tau_pub_.publish(tau_msg_);
    tau_exe_pub_.publish(tau_exe_msg_);
    tau_without_a_pub_.publish(tau_without_a_msg_);
    tau_without_a_v_pub_.publish(tau_without_a_v_msg_);
    a_pub_.publish(a_msg_);
}
void DynamicsInterface::computerInverseDynamics(std::vector<hardware_interface::JointStateHandle> jnt_states)
{
    for (int i = 0; i < (int)tau_.size(); ++i) {
        q_(i) = jnt_states[i].getPosition();
        v_(i) = jnt_states[i].getVelocity();
        double original_a = (v_(i)-last_v_[i])/(ros::Time::now()-last_time_).toSec();
        a_lp_filter_->input(original_a);
        a_(i) = a_lp_filter_->output();
        last_v_[i] = v_(i);
        zero_(i) = 0.;
    }
    last_time_ = ros::Time::now();
//  Use RNEA to computer inverse dynamics
    tau_ = pinocchio::rnea(model_,pinocchio_data_,q_,v_,a_);
    tau_without_a_ = pinocchio::rnea(model_,pinocchio_data_,q_,v_,zero_);
    tau_without_a_v_ = pinocchio::rnea(model_,pinocchio_data_,q_,zero_,zero_);
    for (int i = 0; i < (int)tau_.size(); ++i) {
        tau_error_msg_.data[i] = tau_(i) - jnt_states[i].getEffort();
        tau_msg_.data[i] = tau_(i);
        tau_without_a_msg_.data[i] = tau_without_a_(i);
        tau_without_a_v_msg_.data[i] = tau_without_a_v_(i);
        a_msg_.data[i] = a_(i);
    }
}
}