//
// Created by lsy on 25-3-26.
//

#pragma once

#include "can_device.h"
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <iostream>
#include <memory>
#include <vector>

namespace device {

class CanButtonPanel : public CanDevice {
public:
  CanButtonPanel(const std::string& name, const std::string& bus, int id, const std::string& model, const XmlRpc::XmlRpcValue& config);

  ~CanButtonPanel() override = default;

  can_frame start() override;
  can_frame close() override;
  void read(const can_interface::CanFrameStamp& frameStamp) override;
  void readBuffer(const std::vector<can_interface::CanFrameStamp> &buffer) override;
  can_frame write() override;

  bool isButton1Pressed() const { return button1_pressed_; }
  bool isButton2Pressed() const { return button2_pressed_; }
  double getFrequency() const { return frequency_; }

  // 定时器回调函数，用于检测按钮释放
  void checkButtonsTimeout(const ros::TimerEvent& event);

  bool button1_pressed_;
  bool button2_pressed_;
private:
  ros::NodeHandle nh_;
  ros::Publisher button1_pub_;
  ros::Publisher button2_pub_;
  ros::Timer timeout_timer_;

  double frequency_{0};
  ros::Time last_timestamp_;
  ros::Time button1_last_time_;
  ros::Time button2_last_time_;
  double timeout_; // 按钮释放超时时间（秒）

  void updateFrequency(const ros::Time& stamp);
  void publishButtonState();
  double parseTimeoutFromModel(const std::string& model);
};

} // namespace device
