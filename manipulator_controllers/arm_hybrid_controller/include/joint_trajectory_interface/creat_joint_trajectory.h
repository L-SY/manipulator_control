//
// Created by lsy on 23-11-5.
//

#pragma once

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>

// Project
#include "joint_trajectory_msg_utils.h"
#include "joint_trajectory_segment.h"

namespace arm_hybrid_controller {
    template<class Trajectory>
    bool isNotEmpty(typename Trajectory::value_type trajPerJoint) {
        return !trajPerJoint.empty();
    };

    template<class Trajectory>
    Trajectory creatJointTrajectory(const trajectory_msgs::JointTrajectory &msg,
                                   const ros::Time &time,
                                   std::string &error_string) {
        typedef typename Trajectory::value_type TrajectoryPerJoint;
        typedef typename TrajectoryPerJoint::value_type Segment;

        const unsigned int n_joints = msg.joint_names.size();
        std::vector<std::string> joint_names = msg.joint_names;

        const ros::Time msg_start_time = joint_trajectory_controller::internal::startTime(msg,time); // Message start time

        if (msg.points.empty()) {
            error_string = "Trajectory message contains empty trajectory. Nothing to convert.";
            ROS_DEBUG_STREAM(error_string);
            return Trajectory();
        }

        // Non strictly-monotonic waypoints
        if (!joint_trajectory_controller::isTimeStrictlyIncreasing(msg)) {
            error_string = "Trajectory message contains waypoints that are not strictly increasing in time.";
            ROS_ERROR_STREAM(error_string);
            return Trajectory();
        }
        std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator
                msg_it = joint_trajectory_controller::findPoint(msg,
                                                                time); // Points to last point occurring before current time (NOTE: Using time, not o_time)
        if (msg_it == msg.points.end()) {
            msg_it = msg.points.begin();  // Entire trajectory is after current time
        } else {
            ++msg_it;                     // Points to first point after current time OR sequence end
            if (msg_it == msg.points.end()) {
                ros::Duration last_point_dur = time - (msg_start_time + (--msg_it)->time_from_start);
                error_string = "Dropping all " + std::to_string(msg.points.size());
                error_string += " trajectory point(s), as they occur before the current time.\n";
                error_string += "Last point is " + std::to_string(last_point_dur.toSec());
                error_string += "s in the past.";
                ROS_WARN_STREAM(error_string);
                return Trajectory();
            } else if ( // If the first point is at time zero and no start time is set in the header, skip it silently
                    msg.points.begin()->time_from_start.isZero() &&
                    msg.header.stamp.isZero() &&
                    std::distance(msg.points.begin(), msg_it) == 1
                    ) {
                ROS_DEBUG_STREAM("Dropping first trajectory point at time=0. " <<
                                                                               "First valid point will be reached at time_from_start "
                                                                               <<
                                                                               std::fixed << std::setprecision(3)
                                                                               << msg_it->time_from_start.toSec()
                                                                               << "s.");
            } else {
                ros::Duration next_point_dur = msg_start_time + msg_it->time_from_start - time;
                ROS_WARN_STREAM("Dropping first " << std::distance(msg.points.begin(), msg_it) <<
                                                  " trajectory point(s) out of " << msg.points.size() <<
                                                  ", as they occur before the current time.\n" <<
                                                  "First valid point will be reached in " << std::fixed
                                                  << std::setprecision(3) <<
                                                  next_point_dur.toSec() << "s.");
            }
        }
        Trajectory result_traj; // Currently empty
        result_traj.resize(n_joints);
        for (unsigned int msg_joint_it = 0; msg_joint_it < joint_names.size(); msg_joint_it++) {
            std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = msg_it;
            TrajectoryPerJoint result_traj_per_joint; // Currently empty

            // Add useful segments of new trajectory to result
            // - Construct all trajectory segments occurring after current time
            // - As long as there remain two trajectory points we can construct the next trajectory segment
            while (std::distance(it, msg.points.end()) >= 2) {
                std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator next_it = it;
                ++next_it;
                trajectory_msgs::JointTrajectoryPoint it_point_per_joint, next_it_point_per_joint;
                if (!it->positions.empty()) { it_point_per_joint.positions.resize(1, it->positions[msg_joint_it]); }
                if (!it->velocities.empty()) { it_point_per_joint.velocities.resize(1, it->velocities[msg_joint_it]); }
                if (!it->accelerations.empty()) {
                    it_point_per_joint.accelerations.resize(1, it->accelerations[msg_joint_it]);
                }
                it_point_per_joint.time_from_start = it->time_from_start;
                if (!next_it->positions.empty()) {
                    next_it_point_per_joint.positions.resize(1, next_it->positions[msg_joint_it]);
                }
                if (!next_it->velocities.empty()) {
                    next_it_point_per_joint.velocities.resize(1, next_it->velocities[msg_joint_it]);
                }
                if (!next_it->accelerations.empty()) {
                    next_it_point_per_joint.accelerations.resize(1, next_it->accelerations[msg_joint_it]);
                }
                next_it_point_per_joint.time_from_start = next_it->time_from_start;
                Segment segment(msg_start_time, it_point_per_joint, next_it_point_per_joint);
//            segment.setGoalHandle(options.rt_goal_handle);
                result_traj_per_joint.push_back(segment);
                ++it;
            }
            if (result_traj_per_joint.size() > 0)
                result_traj[msg_joint_it] = result_traj_per_joint;
        }
        return result_traj;
    }
}// namespace arm_hybrid_controller