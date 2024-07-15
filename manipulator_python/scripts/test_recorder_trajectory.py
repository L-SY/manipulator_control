#!/usr/bin/env python

import rospy
from manipulator_python.trajectory_recorder import record_trajectory

def main():
    rospy.init_node('record_trajectory')
    record_trajectory()

if __name__ == '__main__':
    main()
