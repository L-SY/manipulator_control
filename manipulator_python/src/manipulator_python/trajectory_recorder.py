#!/usr/bin/env python

import rospy
import csv
import yaml
from sensor_msgs.msg import JointState

class TrajectoryRecorder:
    def __init__(self, filename):
        self.joint_states = []
        self.filename = filename
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        timestamp = rospy.get_time()
        positions = msg.position
        self.record_state(timestamp, positions)

    def record_state(self, timestamp, positions):
        raise NotImplementedError("Subclasses should implement this method")

    def record_trajectory(self):
        rospy.loginfo("Recording trajectory...")
        rospy.spin()
        self.save_to_file()

    def save_to_file(self):
        raise NotImplementedError("Subclasses should implement this method")

class TrajectoryRecorderCSV(TrajectoryRecorder):
    def __init__(self, filename='recorded_trajectory.csv'):
        super().__init__(filename)

    def record_state(self, timestamp, positions):
        self.joint_states.append((timestamp, positions))

    def save_to_file(self):
        with open(self.filename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['time'] + ['joint_{}'.format(i+1) for i in range(len(self.joint_states[0][1]))])
            for state in self.joint_states:
                row = [state[0]] + list(state[1])
                csvwriter.writerow(row)
        rospy.loginfo("Trajectory saved to {}".format(self.filename))

class TrajectoryRecorderYAML(TrajectoryRecorder):
    def __init__(self, filename='recorded_trajectory.yaml'):
        super().__init__(filename)

    def record_state(self, timestamp, positions):
        self.joint_states.append({'time': timestamp, 'positions': positions})

    def save_to_file(self):
        with open(self.filename, 'w') as yamlfile:
            yaml.dump(self.joint_states, yamlfile)
        rospy.loginfo("Trajectory saved to {}".format(self.filename))
