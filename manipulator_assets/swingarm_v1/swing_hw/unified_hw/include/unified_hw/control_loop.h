//
// Created by lsy on 24-5-21.
//

#pragma once

#include <chrono>
#include <thread>

#include <controller_manager/controller_manager.h>
#include "hardware_interface.h"
#include <ros/ros.h>

namespace control_loop {

class HWControlLoop {  // NOLINT(cppcoreguidelines-special-member-functions)
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;

public:
  /** \brief Create controller manager. Load loop frequency. Start control loop which call @ref
         * legged::RmUnifiedHWLoop::update() in a frequency.
         *
         * @param nh Node-handle of a ROS node.
         * @param hardware_interface A pointer which point to hardware_interface.
   */
  HWControlLoop(ros::NodeHandle& nh, std::shared_ptr<Unified::UnifiedHW> hardware_interface);

  ~HWControlLoop();

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
  std::shared_ptr<Unified::UnifiedHW> hardwareInterface_;
};

}  // namespace control_loop