#include<ros/ros.h>
#include<arm_control/PosCmd.h>

#define PI 3.1415926

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_pub_test_node");
    
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<arm_control::PosCmd>("arx5_pos_cmd", 10);



    //抬升
    arm_control::PosCmd msg1;
    msg1.x = 0.0;
    msg1.y = 0.2;
    msg1.z = 0.2;
    msg1.roll = 0.0;
    msg1.pitch = 0.0;
    msg1.yaw = 0.33*PI;
    //close 0; open_max 5
    msg1.gripper = 5;

    
    //前进
    arm_control::PosCmd msg2;
    msg2.x = 0.1;
    msg2.y = 0.3;
    msg2.z = 0.2;
    msg2.roll = 0.0;
    msg2.pitch = 0.0;
    msg2.yaw = 0.33*PI;
    //close 0; open_max 5
    msg2.gripper = 5;


    //抓取
    arm_control::PosCmd msg3;
    msg3.x = 0.1;
    msg3.y = 0.3;
    msg3.z = 0.2;
    msg3.roll = 0.0;
    msg3.pitch = 0.0;
    msg3.yaw = 0.33*PI;
    //close 0; open_max 5
    msg3.gripper = 0;


    //返回
    arm_control::PosCmd msg4;
    msg4.x = 0.0;
    msg4.y = 0.2;
    msg4.z = 0.2;
    msg4.roll = 0.0;
    msg4.pitch = 0.0;
    msg4.yaw = 0.33*PI;
    //close 0; open_max 5
    msg4.gripper = 0;

    

    //return zero
    arm_control::PosCmd msg0;
    msg0.x = 0.0;
    msg0.y = 0.0;
    msg0.z = 0.0;
    msg0.roll = 0.0;
    msg0.pitch = 0.0;
    msg0.yaw = 0.0;
    //close 0; open_max 5
    msg0.gripper = 0;



    //turn around
    arm_control::PosCmd msg5;
    msg0.x = 0.0;
    msg0.y = 0.0;
    msg0.z = 0.0;
    msg0.roll = 0.0;
    msg0.pitch = 0.0;
    msg0.yaw = PI;
    //close 0; open_max 5
    msg0.gripper = 0;



    //loose object
    arm_control::PosCmd msg6;
    msg0.x = 0.0;
    msg0.y = 0.0;
    msg0.z = 0.0;
    msg0.roll = 0.0;
    msg0.pitch = 0.0;
    msg0.yaw = PI;
    //close 0; open_max 5
    msg0.gripper = 5;


    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        sleep(3);
        printf("msg1\n");
        pub.publish(msg1);

        sleep(3);
        printf("msg2\n");
        pub.publish(msg2);

        sleep(3);
        printf("msg3\n");
        pub.publish(msg3);

        sleep(3);
        printf("msg4\n");
        pub.publish(msg4);


        sleep(3);
        printf("msg0\n");
        pub.publish(msg0);


        sleep(3);
        printf("msg5\n");
        pub.publish(msg5);


        sleep(3);
        printf("msg6\n");
        pub.publish(msg6);

        ros::spin();

        loop_rate.sleep();
    }
    return 0;
}