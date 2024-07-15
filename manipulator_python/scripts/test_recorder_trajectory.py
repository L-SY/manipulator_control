#!/usr/bin/env python

import rospy
from manipulator_python.trajectory_recorder import TrajectoryRecorderCSV, TrajectoryRecorderYAML

def main():
    rospy.init_node('record_trajectory')  # 初始化 ROS 节点

    recorder_type = rospy.get_param('~recorder_type', 'csv')
    output_file = rospy.get_param('~output_file', 'recorded_trajectory.csv')

    if recorder_type == 'csv':
        recorder = TrajectoryRecorderCSV(output_file)
    elif recorder_type == 'yaml':
        recorder = TrajectoryRecorderYAML(output_file)
    else:
        rospy.logerr("Invalid recorder type specified: {}. Use 'csv' or 'yaml'.".format(recorder_type))
        exit(1)

    try:
        recorder.record_trajectory()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
