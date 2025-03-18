import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryClient:
    def __init__(self, print_feedback=False):
        self.client = actionlib.SimpleActionClient('/controllers/arm_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("wait for connect")
        self.client.wait_for_server()
        rospy.loginfo("connect right")
        self.client_feedback = None
        self.print_feedback = print_feedback

    def feedback_callback(self, feedback):
        self.client_feedback = feedback
        if self.print_feedback:
            rospy.loginfo(f"Feedback: {feedback}")

    def send_goal(self, goal):
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo(f"Result: {result}")
        return result

def create_goal(joint_names, trajectory_points):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    points = []
    for point in trajectory_points:
        traj_point = JointTrajectoryPoint()
        traj_point.positions = point['positions']
        traj_point.velocities = point['velocities'] if 'velocities' in point else []
        traj_point.time_from_start = rospy.Duration(point['time_from_start'])
        points.append(traj_point)

    goal.trajectory.points = points

    return goal
