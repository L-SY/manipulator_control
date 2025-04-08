//
// Created by lsy on 24-5-23.
//

#include "gripper_controller/gripper_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace effort_controllers {
/**
   * \brief 夹爪动作控制器，向力矩接口发送命令
 */
typedef gripper_controller::GripperController<hardware_interface::EffortJointInterface>
    GripperController;
} // namespace effort_controllers

namespace position_controllers {
/**
   * \brief 夹爪动作控制器，向位置接口发送命令
 */
typedef gripper_controller::GripperController<hardware_interface::PositionJointInterface>
    GripperController;
} // namespace position_controllers

namespace hybrid_controllers {
/**
   * \brief 夹爪动作控制器，向混合接口发送命令
 */
typedef gripper_controller::GripperController<hardware_interface::HybridJointInterface>
    GripperController;
} // namespace hybrid_controllers

// 注册控制器插件
PLUGINLIB_EXPORT_CLASS(position_controllers::GripperController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(effort_controllers::GripperController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(hybrid_controllers::GripperController, controller_interface::ControllerBase)
