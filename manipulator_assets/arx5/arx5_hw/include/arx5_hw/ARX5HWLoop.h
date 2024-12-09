//
// Created by lsy on 24-9-23.
//

#pragma once

#include "arx5_hw/ARX5HW.h"

#include <chrono>
#include <thread>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

namespace arx5 {

class ARX5HWLoop {  // NOLINT(cppcoreguidelines-special-member-functions)
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;

public:
  /** \brief Create controller manager. Load loop frequency. Start control loop which call @ref
   * arx5::RmRobotHWLoop::update() in a frequency.
   *
   * @param nh Node-handle of a ROS node.
   * @param hardware_interface A pointer which point to hardware_interface.
   */
  ARX5HWLoop(ros::NodeHandle& nh, std::shared_ptr<ARX5HW> hardware_interface);

  ~ARX5HWLoop();

  /** \brief Timed method that reads current hardware's state, runs the controller code once and sends the new commands
   * to the hardware.
   *
   * Timed method that reads current hardware's state, runs the controller code once and sends the new commands to the
   * hardware.
   *
   */
  void update();

private:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Timing
  double cycleTimeErrorThreshold_{}, loopHz_{};
  std::thread loopThread_;
  std::atomic_bool loopRunning_{};
  ros::Duration elapsedTime_;
  Clock::time_point lastTime_;

  /** ROS Controller Manager and Runner

      This class advertises a ROS interface for loading, unloading, starting, and
      stopping ros_control-based controllers. It also serializes execution of all
      running controllers in \ref update.
  **/
  std::shared_ptr<controller_manager::ControllerManager> controllerManager_;

  // Abstract Hardware Interface for your robot
  std::shared_ptr<ARX5HW> hardwareInterface_;
};

}  // namespace arx5


