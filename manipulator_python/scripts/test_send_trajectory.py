#!/usr/bin/env python

import rospy
from manipulator_python.trajectory_client import TrajectoryClient,create_goal

def main():
    rospy.init_node('send_trajectory')
    client = TrajectoryClient()
    goal = create_goal()
    result = client.send_goal(goal)
    rospy.loginfo(result)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
