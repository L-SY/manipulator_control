//
// Created by lsy on 24-5-23.
//

#include "gripper_controller/GripperController.h"

#include <pluginlib/class_list_macros.hpp>

namespace effort_controllers {
  /**
   * \brief Gripper action controller that sends
   * commands to a \b effort interface.
   */
  typedef gripper_controller::gripperController<hardware_interface::EffortJointInterface>
      gripperController;
} // namespace effort_controllers

namespace hybrid_controllers {
/**
   * \brief Gripper action controller that sends
   * commands to a \b effort interface.
 */
  typedef gripper_controller::gripperController<hardware_interface::HybridJointInterface>
      gripperController;
} // namespace hybrid_controllers

PLUGINLIB_EXPORT_CLASS(effort_controllers::gripperController,controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(hybrid_controllers::gripperController,controller_interface::ControllerBase)
