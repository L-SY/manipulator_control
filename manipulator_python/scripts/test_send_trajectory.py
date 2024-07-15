#!/usr/bin/env python

import rospy
from manipulator_python.trajectory_client import TrajectoryClient,create_goal

def main():
    rospy.init_node('send_trajectory')
    client = TrajectoryClient()
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    trajectory_points = [
        {'positions': [0.5, 0.5, -0.5, 0.2, 0.5, 0.5], 'time_from_start': 2.0},
        {'positions': [-0.5, 0.8, -1.0, -0.5, -0.5, -0.5], 'time_from_start': 4.0},
        {'positions': [0., 0., 0., 0., 0., 0.], 'time_from_start': 6.0}
    ]
    goal = create_goal(joint_names, trajectory_points)
    result = client.send_goal(goal)
    rospy.loginfo(result)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
