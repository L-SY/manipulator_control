#!/usr/bin/env python

import rospy
import csv
import yaml
from sensor_msgs.msg import JointState
import os

class TrajectoryRecorder:
    def __init__(self, filename, interval=1.0, threshold=0.01, record_velocity=False):
        self.joint_states = []
        self.new_data = False
        self.filename = filename
        self.interval = interval
        self.threshold = threshold
        self.record_velocity = record_velocity
        self.last_record_time = rospy.Time.now()
        self.start_time = None
        self.last_positions = None
        rospy.on_shutdown(self.save_to_file)

    def initialize_subscriber(self):
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        current_time = rospy.Time.now()
        if self.start_time is None:
            self.start_time = current_time
        relative_time = (current_time - self.start_time).to_sec()

        # 定义需要的关节名称
        required_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        # 找到需要的关节的索引
        indices = [msg.name.index(joint) for joint in required_joints if joint in msg.name]

        # 过滤位置和速度
        positions = [msg.position[i] for i in indices]
        velocities = [msg.velocity[i] for i in indices] if self.record_velocity else None

        if self.last_positions is None:
            rospy.loginfo("Initial record")
            self.last_positions = positions
            self.record_state(relative_time, positions, velocities)
            self.last_record_time = current_time
            self.new_data = True
        else:
            time_diff = (current_time - self.last_record_time).to_sec()
            position_change = max(abs(p1 - p2) for p1, p2 in zip(self.last_positions, positions))

            if time_diff >= self.interval and position_change >= self.threshold:
                self.record_state(relative_time, positions, velocities)
                self.last_record_time = current_time
                self.last_positions = positions
                self.new_data = True

    def record_state(self, timestamp, positions, velocities):
        raise NotImplementedError("Subclasses should implement this method")

    def record_trajectory(self):
        rospy.loginfo("Recording trajectory...")
        self.initialize_subscriber()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def save_to_file(self):
        raise NotImplementedError("Subclasses should implement this method")

class TrajectoryRecorderCSV(TrajectoryRecorder):
    def __init__(self, filename='recorded_trajectory.csv', interval=1.0, threshold=0.01, record_velocity=False):
        super().__init__(filename, interval, threshold, record_velocity)

    def record_state(self, timestamp, positions, velocities):
        self.joint_states.append((timestamp, positions, velocities))
        rospy.loginfo(f"New data recorded: {timestamp}")

    def save_to_file(self):
        if self.new_data:
            rospy.loginfo("Saving to CSV")
            directory = os.path.dirname(self.filename)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
            with open(self.filename, 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                header = ['time'] + ['joint{}_position'.format(i+1) for i in range(len(self.joint_states[0][1]))]
                if self.record_velocity:
                    header += ['joint{}_velocity'.format(i+1) for i in range(len(self.joint_states[0][2]))]
                csvwriter.writerow(header)
                for state in self.joint_states:
                    row = [f"{state[0]:.4f}"] + [f"{pos:.4f}" for pos in state[1]]
                    if self.record_velocity:
                        row += [f"{vel:.4f}" for vel in state[2]]
                    csvwriter.writerow(row)
            rospy.loginfo("Trajectory saved to {}".format(self.filename))
            self.new_data = False  # Reset the flag after saving

class TrajectoryRecorderYAML(TrajectoryRecorder):
    def __init__(self, filename='recorded_trajectory.yaml', interval=1.0, threshold=0.01, record_velocity=False):
        super().__init__(filename, interval, threshold, record_velocity)

    def record_state(self, timestamp, positions, velocities):
        data = {'time': round(timestamp, 4), 'positions': [round(pos, 4) for pos in positions]}
        if self.record_velocity:
            data['velocities'] = [round(vel, 4) for vel in velocities]
        self.joint_states.append(data)
        rospy.loginfo(f"New data recorded: {timestamp}")

    def save_to_file(self):
        if self.new_data:
            rospy.loginfo("Saving to YAML")
            directory = os.path.dirname(self.filename)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)

            formatted_data = {'file_path': [self.filename]}
            for state in self.joint_states:
                formatted_entry = {
                    'time_from_start': state['time'],
                    'positions': state['positions']
                }
                if self.record_velocity and 'velocities' in state:
                    formatted_entry['velocities'] = state['velocities']
                formatted_data['file_path'].append(formatted_entry)

            def represent_list_as_yaml_sequence(dumper, data):
                return dumper.represent_sequence('tag:yaml.org,2002:seq', data)

            yaml.add_representer(list, represent_list_as_yaml_sequence)

            with open(self.filename, 'w') as yamlfile:
                yaml.dump(formatted_data, yamlfile, default_flow_style=False, sort_keys=False)

            rospy.loginfo("Trajectory saved to {}".format(self.filename))
            self.new_data = False  # Reset the flag after saving

if __name__ == '__main__':
    rospy.init_node('trajectory_recorder')
    recorder = TrajectoryRecorderCSV('/path/to/your/directory/recorded_trajectory.csv', record_velocity=True)
    recorder.record_trajectory()
