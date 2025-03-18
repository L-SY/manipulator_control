//
// Created by lsy on 23-11-2.
//
#include <trajectory_interface/quintic_spline_segment.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_quintic_spline_segment");
    ros::NodeHandle test_quintic_spline_segment_nh;
    ros::Publisher q_pub_,v_pub_,a_pub_;
    ros::Publisher contrast_q_pub_,contrast_v_pub_,contrast_a_pub_;
    std_msgs::Float64 q_msg,v_msg,a_msg;
    std_msgs::Float64 contrast_q_msg,contrast_v_msg,contrast_a_msg;
    a_pub_ = test_quintic_spline_segment_nh.advertise<std_msgs::Float64>("a", 1);
    q_pub_ = test_quintic_spline_segment_nh.advertise<std_msgs::Float64>("q", 1);
    v_pub_ = test_quintic_spline_segment_nh.advertise<std_msgs::Float64>("v", 1);
    contrast_a_pub_ = test_quintic_spline_segment_nh.advertise<std_msgs::Float64>("contrast_a", 1);
    contrast_q_pub_ = test_quintic_spline_segment_nh.advertise<std_msgs::Float64>("contrast_q", 1);
    contrast_v_pub_ = test_quintic_spline_segment_nh.advertise<std_msgs::Float64>("contrast_v", 1);
    trajectory_interface::PosVelAccState<double> start_state,end_state,state_store,contrast_state_store;
    start_state.position = {0};
    start_state.velocity = {0};
    start_state.acceleration = {0};
    end_state.position = {10};
    end_state.velocity = {0};
    end_state.acceleration = {0};
    trajectory_interface::QuinticSplineSegment<double> TestQuinticSplineSegment(0.,start_state,10,end_state);
    trajectory_interface::QuinticSplineSegment<double> ContrastTestQuinticSplineSegment(0.,start_state,5,end_state);
    double state_time = ros::Time::now().toSec();

    while (ros::ok())
    {
        ros::spinOnce();
        double sample_time = ros::Time::now().toSec() - state_time;
        TestQuinticSplineSegment.sample(sample_time,state_store);
        ContrastTestQuinticSplineSegment.sample(sample_time,contrast_state_store);
        q_msg.data = state_store.position[0];
        v_msg.data = state_store.velocity[0];
        a_msg.data = state_store.acceleration[0];
        contrast_q_msg.data = contrast_state_store.position[0];
        contrast_v_msg.data = contrast_state_store.velocity[0];
        contrast_a_msg.data = contrast_state_store.acceleration[0];
        q_pub_.publish(q_msg);
        v_pub_.publish(v_msg);
        a_pub_.publish(a_msg);
        contrast_q_pub_.publish(contrast_q_msg);
        contrast_v_pub_.publish(contrast_v_msg);
        contrast_a_pub_.publish(contrast_a_msg);
    }
}