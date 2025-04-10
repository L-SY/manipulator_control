#!/usr/bin/env python
import rospy
import actionlib
import sys
from manipulator_msgs.msg import GripperCommandAction, GripperCommandGoal

def gripper_command(position, max_effort):
    client = actionlib.SimpleActionClient('/controllers/gripper_controller/gripper_cmd', GripperCommandAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Gripper action server not available!")
        return False

    goal = GripperCommandGoal()
    goal.position = position
    goal.max_effort = max_effort

    client.send_goal(goal)
    success = client.wait_for_result(rospy.Duration(10.0))

    if success:
        result = client.get_result()
        rospy.loginfo("Gripper command completed. Success: %s", result.success)
        return result.success
    else:
        rospy.logwarn("Gripper command timed out!")
        return False

if __name__ == '__main__':
    rospy.init_node('gripper_command_client')

    if len(sys.argv) < 2:
        print("Usage: gripper_command.py [open|close|position <value>] [max_effort]")
        sys.exit(1)

    command = sys.argv[1]
    max_effort = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0

    if command == "open":
        gripper_command(0.025, max_effort)  # 完全打开
    elif command == "close":
        gripper_command(0.0, max_effort)    # 完全闭合
    elif command == "position" and len(sys.argv) > 2:
        position = float(sys.argv[2])
        gripper_command(position, max_effort)
    else:
        print("Invalid command. Use 'open', 'close', or 'position <value>'")
