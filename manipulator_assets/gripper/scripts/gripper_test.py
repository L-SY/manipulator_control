#!/usr/bin/env python

import sys
import os
workspace_path = "/home/lsy/manipulator_control"
sys.path.insert(0, os.path.join(workspace_path, "devel/lib/python3/dist-packages"))

import rospy
from manipulator_msgs.msg import GripperState, GripperCmd

class GripperCommandPublisher:
    def __init__(self):
        rospy.init_node('gripper_command_publisher', anonymous=True)

        self.pub = rospy.Publisher('/controllers/gripper_controller/command', GripperCmd, queue_size=10)

        rospy.Subscriber('/controllers/gripper_controller/state', GripperState, self.state_callback)

        self.rate = rospy.Rate(10)

        self.gripper_cmd = GripperCmd()
        self.gripper_cmd.mode = 0
        self.gripper_cmd.des_pos = 0.033
        self.gripper_cmd.des_vel = 1
        self.gripper_cmd.des_eff = 0.0
        self.gripper_cmd.des_kd = 2
        self.gripper_cmd.des_kp = 0.02

        self.current_width = (self.gripper_cmd.des_pos + 0.002) / 2

        self.init_open = False

    def state_callback(self, data):
        self.current_width = data.gripper_pos

    def run(self):
        while not rospy.is_shutdown():
            if not self.init_open:
                self.gripper_cmd.des_pos = 0.002
                self.gripper_cmd.des_vel = 0.1
                if self.current_width < 0.005:
                    self.init_open = True
                    print("init finish")
                print("init")
            elif self.current_width > 0.031:
                self.gripper_cmd.des_pos = 0.002
                self.gripper_cmd.des_vel = -0.1
                print("big")
            elif self.current_width < 0.005:
                self.gripper_cmd.des_pos = 0.033
                self.gripper_cmd.des_vel = 0.1
                print("small")

            self.pub.publish(self.gripper_cmd)
            print(self.current_width)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        gripper_command_publisher = GripperCommandPublisher()
        gripper_command_publisher.run()
    except rospy.ROSInterruptException:
        pass
