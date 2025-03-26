#include <button_position_controller/button_position_controller.h>

namespace button_position_controller
{

ButtonPositionController::ButtonPositionController()
    : error_sum_(0.0)
      , last_error_(0.0)
      , last_button1_state_(false)
      , last_button2_state_(false)
{
}

ButtonPositionController::~ButtonPositionController()
{
}

bool ButtonPositionController::init(hardware_interface::RobotHW* robot_hw,
                                    ros::NodeHandle& nh,
                                    ros::NodeHandle& controller_nh)
{
  // Get parameters
  std::string joint_name;
  if (!controller_nh.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint name specified");
    return false;
  }

  std::string button_panel_name;
  if (!controller_nh.getParam("button_panel", button_panel_name))
  {
    ROS_ERROR("No button panel name specified");
    return false;
  }

  // Get target positions
  controller_nh.param("position_a", position_a_, 0.0);
  controller_nh.param("position_b", position_b_, 1.0);

  // Get PID gains
  controller_nh.param("p_gain", p_gain_, 100.0);
  controller_nh.param("i_gain", i_gain_, 0.1);
  controller_nh.param("d_gain", d_gain_, 10.0);

  // Get joint handle
  try
  {
    auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
    joint_ = effortJointInterface->getHandle(joint_name);
    ROS_INFO_STREAM("Got handle for joint: " << joint_name);
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Error getting joint handle: " << e.what());
    return false;
  }

  // Get button panel handle
  // Note: we need to get it from the appropriate hardware interface
  try
  {
    // We need to get the ButtonPanelInterface from the robot hardware
    // This assumes the robot hardware provides a way to get this interface
    hardware_interface::ButtonPanelInterface* button_interface =
        robot_hw->get<hardware_interface::ButtonPanelInterface>();
    button_panel_ = button_interface->getHandle(button_panel_name);
    if (!button_interface)
    {
      ROS_ERROR("Could not get button panel interface from hardware");
      return false;
    }

    button_panel_ = button_interface->getHandle(button_panel_name);
    ROS_INFO_STREAM("Got handle for button panel: " << button_panel_name);
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Error getting button panel handle: " << e.what());
    return false;
  }

  // Create publishers
  position_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(
      controller_nh, "current_position", 10));

  target_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(
      controller_nh, "target_position", 10));

  button1_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(
      controller_nh, "button1_state", 10));

  button2_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(
      controller_nh, "button2_state", 10));

  ROS_INFO("Button position controller initialized successfully");
  return true;
}

void ButtonPositionController::starting(const ros::Time& time)
{
  // Initialize the controller state
  error_sum_ = 0.0;
  last_error_ = 0.0;

  // Start with the current position as target
  current_target_position_ = joint_.getPosition();

  // Initialize button states
  last_button1_state_ = button_panel_.getButton1State();
  last_button2_state_ = button_panel_.getButton2State();

  ROS_INFO("Button position controller started");
}

void ButtonPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  // Get current button states
  bool button1_state = button_panel_.getButton1State();
  bool button2_state = button_panel_.getButton2State();

  // Check for button state changes
  if (button1_state && !last_button1_state_)
  {
    // Button 1 was just pressed
    current_target_position_ = position_a_;
    ROS_INFO_STREAM("Button 1 pressed, moving to position A: " << position_a_);
  }

  if (button2_state && !last_button2_state_)
  {
    // Button 2 was just pressed
    current_target_position_ = position_b_;
    ROS_INFO_STREAM("Button 2 pressed, moving to position B: " << position_b_);
  }

  // Update button state tracking
  last_button1_state_ = button1_state;
  last_button2_state_ = button2_state;

  // Get current position
  double current_position = joint_.getPosition();

  // Calculate error
  double error = current_target_position_ - current_position;

  // Integrate error
  error_sum_ += error * period.toSec();

  // Calculate derivative of error
  double error_deriv = (error - last_error_) / period.toSec();
  last_error_ = error;

  // PID control
  double command = p_gain_ * error + i_gain_ * error_sum_ + d_gain_ * error_deriv;

  // Set command
  joint_.setCommand(command);

  // Publish status (if possible)
  if (position_pub_ && position_pub_->trylock())
  {
    position_pub_->msg_.data = current_position;
    position_pub_->unlockAndPublish();
  }

  if (target_pub_ && target_pub_->trylock())
  {
    target_pub_->msg_.data = current_target_position_;
    target_pub_->unlockAndPublish();
  }

  if (button1_pub_ && button1_pub_->trylock())
  {
    button1_pub_->msg_.data = button1_state;
    button1_pub_->unlockAndPublish();
  }

  if (button2_pub_ && button2_pub_->trylock())
  {
    button2_pub_->msg_.data = button2_state;
    button2_pub_->unlockAndPublish();
  }
}

void ButtonPositionController::stopping(const ros::Time& time)
{
  // Set command to zero when stopping
  joint_.setCommand(0.0);
  ROS_INFO("Button position controller stopped");
}

} // namespace button_position_controller

// Register controller plugin
PLUGINLIB_EXPORT_CLASS(button_position_controller::ButtonPositionController, controller_interface::ControllerBase)
