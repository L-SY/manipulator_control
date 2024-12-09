//
// Created by lsy on 24-9-23.
//

#include "arx5_hw/ARX5HW.h"

#include "arx5_hw/ARX5HWLoop.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "arx5_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~");

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try {
    // Create the hardware interface specific to your robot
    std::shared_ptr<arx5::ARX5HW> ARX5HW = std::make_shared<arx5::ARX5HW>();
    // Initialize the hardware interface:
    // 1. retrieve configuration from rosparam
    // 2. initialize the hardware and interface it with ros_control
    ARX5HW->init(nh, robotHwNh);

    // Start the control loop
    arx5::ARX5HWLoop controlLoop(nh, ARX5HW);

    // Wait until shutdown signal received
    ros::waitForShutdown();
  } catch (const ros::Exception& e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}