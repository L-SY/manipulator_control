#!/usr/bin/env python

import rospy
import os
from trajectory_tools.trajectory_client import TrajectoryClient,create_goal
from trajectory_tools.trajectory_reader import read_trajectory_from_csv,read_trajectory_from_yaml

def main():
    rospy.init_node('send_trajectory')
    client = TrajectoryClient()
    # Test for send custom data
    # joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    # trajectory_points = [
    #     {'positions': [0., 0., 0., 0., 0., 0.], 'velocities': [0.1, 0.1, -0.1, 0.1, 0.1, 0.1], 'time_from_start': 2.0},
    #     {'positions': [0.5, 0.5, -0.5, 0.2, 0.5, 0.5], 'velocities': [0.1, 0.1, -0.1, 0.1, 0.1, 0.1], 'time_from_start': 4.0},
    #     {'positions': [-0.5, 0.8, -1.0, -0.5, -0.5, -0.5], 'velocities': [-0.1, 0.2, -0.2, -0.1, -0.1, -0.1], 'time_from_start': 6.0},
    #     {'positions': [0., 0., 0., 0., 0., 0.], 'velocities': [0., 0., 0., 0., 0., 0.], 'time_from_start': 8.0}
    # ]

    # file_path = '/home/lsy/manipulator_control/src/manipulator_python/trajectory/recorded_trajectory.csv'
    # joint_names, trajectory_points = read_trajectory_from_csv(file_path)

    # file_path = '/home/lsy/manipulator_control/src/manipulator_python/trajectory/recorded_trajectory.yaml'
    # joint_names, trajectory_points = read_trajectory_from_yaml(file_path)

    file_type = rospy.get_param('~file_type', 'yaml')
    file_path = rospy.get_param('~file_path', '../trajectory/recorded_trajectory.yaml')

    if not os.path.isabs(file_path):
        file_path = os.path.join(os.path.dirname(__file__), file_path)
    if not os.path.exists(file_path):
        rospy.logerr(f"File not found: {file_path}")
        return

    client = TrajectoryClient()

    if file_type == 'csv':
        joint_names, trajectory_points = read_trajectory_from_csv(file_path)
    elif file_type == 'yaml':
        joint_names, trajectory_points = read_trajectory_from_yaml(file_path)
    else:
        rospy.logerr("Invalid file type specified: {}. Use 'csv' or 'yaml'.".format(file_type))
        return

    goal = create_goal(joint_names, trajectory_points)
    result = client.send_goal(goal)
    rospy.loginfo(result)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
