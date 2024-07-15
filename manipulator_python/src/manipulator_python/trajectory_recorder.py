#!/usr/bin/env python

import rospy
import csv
import yaml
from sensor_msgs.msg import JointState

class TrajectoryRecorderCSV:
    def __init__(self, filename='recorded_trajectory.csv'):
        self.joint_states = []
        self.filename = filename
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        timestamp = rospy.get_time()
        positions = msg.position
        self.joint_states.append((timestamp, positions))

    def record_trajectory(self):
        rospy.loginfo("Recording trajectory...")
        rospy.spin()
        self.save_to_file()

    def save_to_file(self):
        with open(self.filename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['time'] + ['joint_{}'.format(i+1) for i in range(len(self.joint_states[0][1]))])
            for state in self.joint_states:
                row = [state[0]] + list(state[1])
                csvwriter.writerow(row)
        rospy.loginfo("Trajectory saved to {}".format(self.filename))

class TrajectoryRecorderYAML:
    def __init__(self, filename='recorded_trajectory.yaml'):
        self.joint_states = []
        self.filename = filename
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        timestamp = rospy.get_time()
        positions = msg.position
        self.joint_states.append({'time': timestamp, 'positions': positions})

    def record_trajectory(self):
        rospy.loginfo("Recording trajectory...")
        rospy.spin()
        self.save_to_file()

    def save_to_file(self):
        with open(self.filename, 'w') as yamlfile:
            yaml.dump(self.joint_states, yamlfile)
        rospy.loginfo("Trajectory saved to {}".format(self.filename))

def record_trajectory():
    recorder_type = rospy.get_param('~recorder_type', 'csv')
    output_file = rospy.get_param('~output_file', 'recorded_trajectory.csv')

    if recorder_type == 'csv':
        recorder = TrajectoryRecorderCSV(output_file)
    elif recorder_type == 'yaml':
        recorder = TrajectoryRecorderYAML(output_file)
    else:
        rospy.logerr("Invalid recorder type specified: {}. Use 'csv' or 'yaml'.")
        exit(1)

    try:
        recorder.record_trajectory()
    except rospy.ROSInterruptException:
        pass
