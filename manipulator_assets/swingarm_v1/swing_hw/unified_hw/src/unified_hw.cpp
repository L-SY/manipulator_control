//
// Created by lsy on 24-12.9
//

#include "unified_hw/hardware_interface.h"
#include "unified_hw/control_loop.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unified_hw");
  ros::NodeHandle nh;

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(2);
  spinner.start();
  try
  {
    // Create the hardware interface specific to your robot
    std::shared_ptr<Unified::UnifiedHW> unified_hw_interface = std::make_shared<Unified::UnifiedHW>();
    // Start the control loop
    control_loop::HWControlLoop control_loop(nh, unified_hw_interface);

    // Wait until shutdown signal received
    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }
  return 0;
}
