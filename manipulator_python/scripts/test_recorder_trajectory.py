#!/usr/bin/env python

import rospy
from manipulator_python.trajectory_recorder import TrajectoryRecorderCSV, TrajectoryRecorderYAML

def main():
    rospy.init_node('record_trajectory')  # 初始化 ROS 节点

    recorder_type = rospy.get_param('~recorder_type', 'csv')
    output_file = rospy.get_param('~output_file', '../trajectory/recorded_trajectory.csv')
    interval = rospy.get_param('~interval', 0.5)
    threshold = rospy.get_param('~threshold', 0.01)

    if recorder_type == 'csv':
        recorder = TrajectoryRecorderCSV(output_file, interval, threshold)
    elif recorder_type == 'yaml':
        recorder = TrajectoryRecorderYAML(output_file, interval, threshold)
    else:
        rospy.logerr("Invalid recorder type specified: {}. Use 'csv' or 'yaml'.".format(recorder_type))
        exit(1)

    try:
        recorder.record_trajectory()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
