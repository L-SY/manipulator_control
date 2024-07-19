#!/usr/bin/env python

import rospy
import csv
import yaml
from sensor_msgs.msg import JointState
import os

class TrajectoryRecorder:
    def __init__(self, filename, interval=1.0, threshold=0.01):
        self.joint_states = []
        self.new_data = False
        self.filename = filename
        self.interval = interval
        self.threshold = threshold
        self.last_record_time = rospy.Time.now()
        self.start_time = None
        self.last_positions = None
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.on_shutdown(self.save_to_file)

    def joint_state_callback(self, msg):
        current_time = rospy.Time.now()
        if self.start_time is None:
            self.start_time = current_time
        relative_time = (current_time - self.start_time).to_sec()
        positions = msg.position

        if self.last_positions is None:
            rospy.loginfo("Initial record")
            self.last_positions = positions
            self.record_state(relative_time, positions)
            self.last_record_time = current_time
            self.new_data = True
        else:
            time_diff = (current_time - self.last_record_time).to_sec()
            position_change = max(abs(p1 - p2) for p1, p2 in zip(self.last_positions, positions))

            if time_diff >= self.interval and position_change >= self.threshold:
                self.record_state(relative_time, positions)
                self.last_record_time = current_time
                self.last_positions = positions
                self.new_data = True

    def record_state(self, timestamp, positions):
        raise NotImplementedError("Subclasses should implement this method")

    def record_trajectory(self):
        rospy.loginfo("Recording trajectory...")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def save_to_file(self):
        raise NotImplementedError("Subclasses should implement this method")

class TrajectoryRecorderCSV(TrajectoryRecorder):
    def __init__(self, filename='recorded_trajectory.csv', interval=1.0, threshold=0.01):
        super().__init__(filename, interval, threshold)

    def record_state(self, timestamp, positions):
        self.joint_states.append((timestamp, positions))

    def save_to_file(self):
        if self.new_data:
            rospy.loginfo("Saving to CSV")
            directory = os.path.dirname(self.filename)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
            with open(self.filename, 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(['time_from_start'] + ['joint_{}'.format(i+1) for i in range(len(self.joint_states[0][1]))])
                for state in self.joint_states:
                    row = [state[0]] + list(state[1])
                    csvwriter.writerow(row)
            rospy.loginfo("Trajectory saved to {}".format(self.filename))
            self.new_data = False  # Reset the flag after saving

class TrajectoryRecorderYAML(TrajectoryRecorder):
    def __init__(self, filename='recorded_trajectory.yaml', interval=1.0, threshold=0.01):
        super().__init__(filename, interval, threshold)

    def record_state(self, timestamp, positions):
        self.joint_states.append({'time': timestamp, 'positions': positions})
        rospy.loginfo(f"New data recorded: {timestamp}, {positions}")

    def save_to_file(self):
        if self.new_data:
            rospy.loginfo("Saving to YAML")
            directory = os.path.dirname(self.filename)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
            with open(self.filename, 'w') as yamlfile:
                yaml.dump(self.joint_states, yamlfile)
            rospy.loginfo("Trajectory saved to {}".format(self.filename))
            self.new_data = False  # Reset the flag after saving

if __name__ == '__main__':
    rospy.init_node('trajectory_recorder')
    recorder = TrajectoryRecorderCSV('/path/to/your/directory/recorded_trajectory.csv')
    recorder.record_trajectory()
