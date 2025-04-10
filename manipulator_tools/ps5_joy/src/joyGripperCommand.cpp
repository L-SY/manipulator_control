//
// Created by lsy on 25-4-10.
//

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "ps5_joy/ps5Joy.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_rl");
  ros::NodeHandle nh;

  joy::PS5Joy ps5_joy(nh);

  std::string topic_name;
  int frequency;
  nh.param("rl_topic", topic_name, std::string("/controllers/gripper_controller/normalized_command"));
  nh.param("frequency", frequency, 100);
  ros::Publisher gripper_pub = nh.advertise<std_msgs::Float64>(topic_name, 10);

  ros::Rate rate(frequency);

  while (ros::ok()) {
    std_msgs::Float64 cmd_msg;
    cmd_msg.data = ps5_joy.getAxisValue(joy::PS5ButtonMap::R2Pressure);
    gripper_pub.publish(cmd_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
