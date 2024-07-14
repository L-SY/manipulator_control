import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/controllers/arm_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def send_goal(self, goal):
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()

def create_goal():
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    point1 = JointTrajectoryPoint()
    point1.positions = [0.5, 0.5, -0.5, 0.2, 0.5, 0.5]
    point1.time_from_start = rospy.Duration(2.0)

    point2 = JointTrajectoryPoint()
    point2.positions = [-0.5, 0.8, -1.0, -0.5, -0.5, -0.5]
    point2.time_from_start = rospy.Duration(4.0)

    goal.trajectory.points = [point1, point2]

    return goal
