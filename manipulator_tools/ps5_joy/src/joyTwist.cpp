//
// Created by lsy on 24-9-12.
//

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "ps5_joy/ps5Joy.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_twist");
  ros::NodeHandle nh;

  joy::PS5Joy ps5_joy(nh);

  std::string topic_name;
  int frequency;
  bool add_stamp;
  double linear_x_limit, angular_z_limit, linear_z_limit;
  double linear_y_limit, angular_x_limit, angular_y_limit;

  nh.param("twist_topic", topic_name, std::string("/cmd_vel"));
  nh.param("frequency", frequency, 100);
  nh.param("add_stamp", add_stamp, false);
  nh.param("linear_x_limit", linear_x_limit, 5.0);
  nh.param("linear_y_limit", linear_y_limit, 0.5);
  nh.param("linear_z_limit", linear_z_limit, 0.4);
  nh.param("angular_x_limit", angular_x_limit, 1.0);
  nh.param("angular_y_limit", angular_y_limit, 1.0);
  nh.param("angular_z_limit", angular_z_limit, 3.14);

  ros::Publisher twist_pub;
  if (add_stamp) {
    twist_pub = nh.advertise<geometry_msgs::TwistStamped>(topic_name, 10);
  } else {
    twist_pub = nh.advertise<geometry_msgs::Twist>(topic_name, 10);
  }

  ros::Rate rate(frequency);

  while (ros::ok()) {
    int line_direction =  ps5_joy.getButtonState(joy::PS5ButtonMap::L1) ? 1 : -1;
    int angular_direction =  ps5_joy.getButtonState(joy::PS5ButtonMap::R1) ? 1 : -1;
    if (add_stamp) {
      geometry_msgs::TwistStamped twist_stamped_msg;
      twist_stamped_msg.header.stamp = ros::Time::now();

      twist_stamped_msg.twist.linear.x = linear_x_limit * ps5_joy.getAxisValue(joy::PS5ButtonMap::L3Vertical);
      twist_stamped_msg.twist.linear.y = linear_y_limit * ps5_joy.getAxisValue(joy::PS5ButtonMap::L3Horizontal);
      twist_stamped_msg.twist.linear.z = linear_z_limit * line_direction * (ps5_joy.getAxisValue(joy::PS5ButtonMap::L2Pressure)-1) / 2;
      twist_stamped_msg.twist.angular.x = angular_x_limit * ps5_joy.getAxisValue(joy::PS5ButtonMap::R3Vertical);
      twist_stamped_msg.twist.angular.y = angular_y_limit * ps5_joy.getAxisValue(joy::PS5ButtonMap::R3Horizontal);
      twist_stamped_msg.twist.angular.z = angular_z_limit * angular_direction * (ps5_joy.getAxisValue(joy::PS5ButtonMap::R2Pressure)-1) / 2;

      twist_pub.publish(twist_stamped_msg);
    } else {
      geometry_msgs::Twist twist_msg;

      twist_msg.linear.x = linear_x_limit * ps5_joy.getAxisValue(joy::PS5ButtonMap::L3Vertical);
      twist_msg.linear.y = linear_y_limit * ps5_joy.getAxisValue(joy::PS5ButtonMap::L3Horizontal);
      twist_msg.linear.z = linear_z_limit * line_direction * (ps5_joy.getAxisValue(joy::PS5ButtonMap::L2Pressure)-1) / 2;
      twist_msg.angular.x = angular_x_limit * ps5_joy.getAxisValue(joy::PS5ButtonMap::R3Vertical);
      twist_msg.angular.y = angular_y_limit * ps5_joy.getAxisValue(joy::PS5ButtonMap::R3Horizontal);
      twist_msg.angular.z = angular_z_limit * angular_direction * (ps5_joy.getAxisValue(joy::PS5ButtonMap::R2Pressure)-1) / 2;

      twist_pub.publish(twist_msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

    return 0;
}
