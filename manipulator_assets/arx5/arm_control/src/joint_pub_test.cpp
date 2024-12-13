#include<ros/ros.h>
#include<arm_control/JointControl.h>

#define PI 3.1415926

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joint_pub_test_node");
    
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<arm_control::JointControl>("joint_control", 10);

    arm_control::JointControl msg1;
    msg1.joint_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 1.0};
    msg1.joint_vel = {0};
    msg1.joint_cur = {0};


    // ros::Rate loop_rate(1);

    // while (ros::ok())
    // {
        sleep(1);
        printf("msg1\n");
        pub.publish(msg1);

        ros::spin();

    //     loop_rate.sleep();
    // }
    return 0;
}