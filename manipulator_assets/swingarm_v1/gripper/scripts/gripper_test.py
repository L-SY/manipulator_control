#!/usr/bin/env python

import sys
import os
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class GripperCommandPublisher:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('gripper_command_publisher', anonymous=True)

        # 发布器：发布夹爪目标位置到控制器
        self.pub = rospy.Publisher('/controllers/gripper_controller/command', Float64, queue_size=10)

        # 订阅器：订阅夹爪的状态数据
        rospy.Subscriber('/joint_states', JointState, self.state_callback)

        # 发布频率
        self.rate = rospy.Rate(10)

        # 初始化目标位置
        self.des_pos = 0.033  # 默认目标位置
        self.current_width = 0.0  # 当前夹爪位置

        # 初始化状态
        self.init_open = False

    def state_callback(self, data):
        # 从 /joint_states 获取夹爪的当前位置
        try:
            joint_index = data.name.index("left_gripper_joint")  # 根据具体关节名称修改
            self.current_width = data.position[joint_index]
        except ValueError:
            rospy.logwarn("gripper_joint not found in /joint_states")

    def run(self):
        while not rospy.is_shutdown():
            # 初始化打开夹爪
            if not self.init_open:
                self.des_pos = 0.002
                if self.current_width < 0.005:
                    self.init_open = True
                    rospy.loginfo("Initialization finished")
                rospy.loginfo("Initializing...")
            # 如果夹爪过大，收回
            elif self.current_width > 0.031:
                self.des_pos = 0.002
                rospy.loginfo("Gripper too wide, closing")
            # 如果夹爪过小，打开
            elif self.current_width < 0.005:
                self.des_pos = 0.033
                rospy.loginfo("Gripper too narrow, opening")

            # 发布目标位置
            self.pub.publish(self.des_pos)
            rospy.loginfo("Current width: {:.4f}, Target position: {:.4f}".format(self.current_width, self.des_pos))

            # 按频率休眠
            self.rate.sleep()

if __name__ == '__main__':
    try:
        gripper_command_publisher = GripperCommandPublisher()
        gripper_command_publisher.run()
    except rospy.ROSInterruptException:
        pass
