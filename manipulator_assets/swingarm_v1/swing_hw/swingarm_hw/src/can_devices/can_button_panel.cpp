//
// Created by lsy on 25-3-26.
//

#include "swingarm_hw/can_devices/can_button_panel.h"
#include "ros/ros.h"

namespace device {

CanButtonPanel::CanButtonPanel(const std::string& name, const std::string& bus, const int id, const std::string& model, const XmlRpc::XmlRpcValue& config)
    : CanDevice(name, bus, id, model, DeviceType::only_read, config),
      button1_pressed_(false),
      button2_pressed_(false)
{
  // 从model字符串解析超时时间
  timeout_ = parseTimeoutFromModel(model) / 1000.0; // 转换为秒

  last_timestamp_ = ros::Time::now();
  button1_last_time_ = ros::Time::now();
  button2_last_time_ = ros::Time::now();

  // 创建ROS发布者
  button1_pub_ = nh_.advertise<std_msgs::Bool>(name + "/button1", 10);
  button2_pub_ = nh_.advertise<std_msgs::Bool>(name + "/button2", 10);

  // 创建定时器，用于检测按钮释放
  timeout_timer_ = nh_.createTimer(ros::Duration(0.05), &CanButtonPanel::checkButtonsTimeout, this);

  ROS_INFO_STREAM("Button panel device initialized: " << getName() << ", timeout: " << timeout_ << "s");
}

double CanButtonPanel::parseTimeoutFromModel(const std::string& model) {
  // 默认超时时间为500毫秒
  double timeout_ms = 500.0;

  // 提取model字符串中的数字部分
  std::string number_str;
  for (char c : model) {
    if (std::isdigit(c)) {
      number_str += c;
    }
  }

  // 如果找到数字，则转换为超时时间
  if (!number_str.empty()) {
    try {
      timeout_ms = std::stod(number_str);
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("Failed to parse timeout from model " << model << ": " << e.what());
    }
  }

  ROS_INFO_STREAM("Parsed timeout from model " << model << ": " << timeout_ms << " ms");
  return timeout_ms;
}

can_frame CanButtonPanel::start() {
  ROS_INFO_STREAM("Starting button panel device: " << getName());
  can_frame frame;
  for (int i = 0; i < 8; i++)
  {
    frame.data[i] = 0x00;
  }
  return frame;
}

can_frame CanButtonPanel::close() {
  ROS_INFO_STREAM("Closing button panel device: " << getName());
  can_frame frame;
  for (int i = 0; i < 8; i++)
  {
    frame.data[i] = 0x00;
  }
  return frame;
}

can_frame CanButtonPanel::write() {
  ROS_DEBUG_STREAM("Button panel is read-only device, write operation not supported");
  can_frame frame;
  for (int i = 0; i < 8; i++)
  {
    frame.data[i] = 0x00;
  }
  return frame;
}

void CanButtonPanel::updateFrequency(const ros::Time& stamp) {
  double dt = (stamp - last_timestamp_).toSec();
  if (dt > 0)
    frequency_ = 1.0 / dt;
  else
    frequency_ = 0;
}

void CanButtonPanel::read(const can_interface::CanFrameStamp& frameStamp) {
  ros::Time current_time = ros::Time::now();
  auto frame = frameStamp.frame;

  // 检测按钮1状态
  if(frame.data[0] == 0x00 && frame.data[1] == 0x01) {
    button1_pressed_ = true;
    button1_last_time_ = current_time;
    ROS_DEBUG_STREAM("Button 1 pressed on device: " << getName());
  }

  // 检测按钮2状态
  if(frame.data[0] == 0x01 && frame.data[1] == 0x00) {
    button2_pressed_ = true;
    button2_last_time_ = current_time;
    ROS_DEBUG_STREAM("Button 2 pressed on device: " << getName());
  }

  last_timestamp_ = current_time;
  updateFrequency(last_timestamp_);
  publishButtonState();
}

void CanButtonPanel::readBuffer(const std::vector<can_interface::CanFrameStamp>& frameStamps)
{
  for (const auto& frame : frameStamps) {
    if (frame.frame.can_id == getId()) {
      read(frame);
    }
  }
}

void CanButtonPanel::checkButtonsTimeout(const ros::TimerEvent& event) {
  ros::Time current_time = ros::Time::now();
  bool state_changed = false;

  // 检查按钮1是否超时（释放）
  if (button1_pressed_ && (current_time - button1_last_time_).toSec() > timeout_) {
    button1_pressed_ = false;
    state_changed = true;
    ROS_DEBUG_STREAM("Button 1 released (timeout) on device: " << getName());
  }

  // 检查按钮2是否超时（释放）
  if (button2_pressed_ && (current_time - button2_last_time_).toSec() > timeout_) {
    button2_pressed_ = false;
    state_changed = true;
    ROS_DEBUG_STREAM("Button 2 released (timeout) on device: " << getName());
  }

  // 如果状态有变化，发布最新状态
  if (state_changed) {
    publishButtonState();
  }
}

void CanButtonPanel::publishButtonState() {
  std_msgs::Bool button1_msg;
  std_msgs::Bool button2_msg;

  button1_msg.data = button1_pressed_;
  button2_msg.data = button2_pressed_;

  button1_pub_.publish(button1_msg);
  button2_pub_.publish(button2_msg);
}

} // namespace device
