#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class GripperCommandPublisher:
    def __init__(self):
        # 初始化节点
        rospy.init_node('gripper_command_publisher', anonymous=True)

        # 创建发布者，发布到 /controllers/gripper_controller/command 话题
        self.pub = rospy.Publisher('/controllers/gripper_controller/command', Float64, queue_size=10)

        # 订阅 /controllers/gripper_controller/state 话题
        rospy.Subscriber('/controllers/gripper_controller/state', Float64, self.state_callback)

        # 设置发布频率为 10Hz
        self.rate = rospy.Rate(100)

        # 初始化命令消息
        self.gripper_open = Float64()
        self.gripper_close = Float64()
        self.gripper_open.data = 0.033
        self.gripper_close.data = 0.002

        # 夹爪当前宽度
        self.current_width = (self.gripper_open.data+self.gripper_close.data)/2

        # 切换标志
        self.init_open = False

    def state_callback(self, data):
        self.current_width = data.data

    def run(self):
        while not rospy.is_shutdown():
            if not self.init_open:
                self.pub.publish(self.gripper_close)
                if self.current_width < 0.005:
                    self.init_open = True
            if self.current_width > 0.031:
                self.pub.publish(self.gripper_close)
            if self.current_width < 0.005:
                self.pub.publish(self.gripper_open)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        gripper_command_publisher = GripperCommandPublisher()
        gripper_command_publisher.run()
    except rospy.ROSInterruptException:
        pass
