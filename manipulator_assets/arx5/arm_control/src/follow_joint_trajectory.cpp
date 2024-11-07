# include <ros/ros.h>
# include <actionlib/server/simple_action_server.h>
# include <control_msgs/FollowJointTrajectoryAction.h>
# include <std_msgs/Float32MultiArray.h>
# include <iostream>
# include <moveit_msgs/RobotTrajectory.h>
# include <arm_control/JointControl.h>

using namespace std;
// 重命名类型为 Server
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

// 用于存储 moveit 发送出来的轨迹数据
moveit_msgs::RobotTrajectory moveit_tra;

//create publisher
ros::Publisher pub;

void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goalPtr, Server* moveit_server)
{
    // 1、解析提交的目标值
    int n_joints = goalPtr->trajectory.joint_names.size();
    int n_tra_Points = goalPtr->trajectory.points.size();

    moveit_tra.joint_trajectory.header.frame_id = goalPtr->trajectory.header.frame_id;
    moveit_tra.joint_trajectory.joint_names = goalPtr->trajectory.joint_names;
    moveit_tra.joint_trajectory.points.resize(n_tra_Points);

    for(int i=0; i<n_tra_Points; i++) // 遍历每组路点
    {
        moveit_tra.joint_trajectory.points[i].positions.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].velocities.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].accelerations.resize(n_joints);

        moveit_tra.joint_trajectory.points[i].time_from_start = goalPtr->trajectory.points[i].time_from_start;
        for(int j=0;j<n_joints; j++) // 遍历每组路点中的每个关节数据
        {
            moveit_tra.joint_trajectory.points[i].positions[j] = goalPtr->trajectory.points[i].positions[j];
            moveit_tra.joint_trajectory.points[i].velocities[j] = goalPtr->trajectory.points[i].velocities[j];
            moveit_tra.joint_trajectory.points[i].accelerations[j] = goalPtr->trajectory.points[i].accelerations[j];
        }
    }

    cout << "The trajectory data is:" << "********************************************" << endl;
    cout << moveit_tra;
    cout << "********************************************" << "The trajectory data is finished printing." << endl;
    ROS_INFO("The number of joints is %d.",n_joints);
    ROS_INFO("The waypoints number of the trajectory is %d.",n_tra_Points);

    ROS_INFO("Receive trajectory successfully");
    moveit_server->setSucceeded();

    arm_control::JointControl msg;
    for(int i = 0; i < moveit_tra.joint_trajectory.points.size(); i++)
    {
        for(int j = 0; j < n_joints; j++)
        {
            msg.joint_pos[j] = moveit_tra.joint_trajectory.points[i].positions[j];
            msg.joint_vel[j] = 0.0;
            msg.joint_cur[j] = 0.0;

            printf("msg.joint_pos: %f \n", msg.joint_pos[j]);
        }
        //sleep(1);
        printf("msg\n");
        pub.publish(msg);
    }


}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"moveit_action_server");
    ros::NodeHandle nh;

    pub = nh.advertise<arm_control::JointControl>("joint_control", 10);

    // 创建 action 对象(NodeHandle，话题名称，回调函数解析传入的目标值，服务器是否自启动)
    Server moveit_server(nh,"arx5_arm_controller/follow_joint_trajectory", boost::bind(&execute_callback, _1, &moveit_server), false);
    // 手动启动服务器
    moveit_server.start();
    
        

    ros::spin();

    return 0;

}
