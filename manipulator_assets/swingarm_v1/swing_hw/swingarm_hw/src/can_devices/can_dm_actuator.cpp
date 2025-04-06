//
// Created by lsy on 24-12-16.
//

// can_dm_actuator.cpp
#include "swingarm_hw/can_devices/can_dm_actuator.h"

namespace device {

CanDmActuator::CanDmActuator(const std::string& name, const std::string& bus, int id, const std::string& motor_type, const XmlRpc::XmlRpcValue& config)
    : CanDevice(name, bus, id, motor_type, DeviceType::read_write, config), coeff_(ActuatorConfig().getActuatorCoefficients(motor_type))
{
  // 从配置中读取控制模式，默认为EFFORT
  if (config.hasMember("control_mode")) {
    std::string mode_str = static_cast<std::string>(config["control_mode"]);
    if (mode_str == "MIT") {
      control_mode_ = ControlMode::MIT;
    } else if (mode_str == "POSITION_VELOCITY") {
      control_mode_ = ControlMode::POSITION_VELOCITY;
    } else {
      control_mode_ = ControlMode::EFFORT; // 默认或明确指定为EFFORT
    }
  }
}

can_frame CanDmActuator::start()
{
  can_frame frame{};
  frame.can_id = id_;
  frame.can_dlc = 8;

  for (int i = 0; i < 7; i++)
  {
    frame.data[i] = 0xFF;
  }
  frame.data[7] = 0xFC;
  return frame;
}

can_frame CanDmActuator::close()
{
  can_frame frame{};
  frame.can_id = id_;
  frame.can_dlc = 8;

  for (int i = 0; i < 7; i++)
  {
    frame.data[i] = 0xFF;
  }
  frame.data[7] = 0xFD;
  return frame;
}

void CanDmActuator::read(const can_interface::CanFrameStamp& frameStamp)
{
  auto frame = frameStamp.frame;
  uint16_t q_raw = (frame.data[1] << 8) | frame.data[2];
  uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
  uint16_t cur = ((frame.data[4] & 0xF) << 8) | frame.data[5];

  position_ = coeff_.act2pos * static_cast<double>(q_raw) + coeff_.pos_offset;
  velocity_ = coeff_.act2vel * static_cast<double>(qd) + coeff_.vel_offset;
  effort_ = coeff_.act2effort * static_cast<double>(cur) + coeff_.effort_offset;

  ros::Time current_time = ros::Time::now();
  updateFrequency(current_time);
  last_timestamp_ = current_time;
  seq_++;
}

void CanDmActuator::readBuffer(const std::vector<can_interface::CanFrameStamp>& frameStamps)
{
  for (const auto& frameStamp : frameStamps) {
    if (frameStamp.frame.can_id == 0x000) {
      if ((frameStamp.frame.data[0] & 0b00001111) == id_) {
        read(frameStamp);
        break;
      }
    }
  }
}

can_frame CanDmActuator::write()
{
  can_frame frame;

  // 根据控制模式选择不同的写入方法
  if (control_mode_ == ControlMode::POSITION_VELOCITY) {
    frame = writePositionVelocity();
  } else {
    // EFFORT和MIT模式使用相同的写入方法
    frame = writeEffortMIT();
  }

  delayMicroseconds(100);

  return frame;
}

can_frame CanDmActuator::writeEffortMIT()
{
  can_frame frame{};
  frame.can_id = id_;
  frame.can_dlc = 8;

  uint16_t q_des = static_cast<int>(coeff_.pos2act * (cmd_position_ - coeff_.pos_offset));
  uint16_t qd_des = static_cast<int>(coeff_.vel2act * (cmd_velocity_ - coeff_.vel_offset));
  uint16_t kp = static_cast<int>(coeff_.kp2act * cmd_kp_);
  uint16_t kd = static_cast<int>(coeff_.kd2act * cmd_kd_);
  uint16_t tau = static_cast<int>(coeff_.effort2act * (cmd_effort_ - coeff_.effort_offset));

  frame.data[0] = q_des >> 8;
  frame.data[1] = q_des & 0xFF;
  frame.data[2] = qd_des >> 4;
  frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
  frame.data[4] = kp & 0xFF;
  frame.data[5] = kd >> 4;
  frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
  frame.data[7] = tau & 0xFF;

  return frame;
}

can_frame CanDmActuator::writePositionVelocity()
{
  // TODO: 实现POSITION_VELOCITY模式的写入逻辑
  // 目前先返回与EFFORT/MIT相同的帧，后续会根据需求修改
  can_frame frame{};
  frame.can_id = id_;
  frame.can_dlc = 8;

  uint16_t q_des = static_cast<int>(coeff_.pos2act * (cmd_position_ - coeff_.pos_offset));
  uint16_t qd_des = static_cast<int>(coeff_.vel2act * (cmd_velocity_ - coeff_.vel_offset));
  uint16_t kp = static_cast<int>(coeff_.kp2act * cmd_kp_);
  uint16_t kd = static_cast<int>(coeff_.kd2act * cmd_kd_);
  uint16_t tau = static_cast<int>(coeff_.effort2act * (cmd_effort_ - coeff_.effort_offset));

  frame.data[0] = q_des >> 8;
  frame.data[1] = q_des & 0xFF;
  frame.data[2] = qd_des >> 4;
  frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
  frame.data[4] = kp & 0xFF;
  frame.data[5] = kd >> 4;
  frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
  frame.data[7] = tau & 0xFF;

  return frame;
}

void CanDmActuator::setCommand(double pos, double vel, double kp, double kd, double effort)
{
  cmd_position_ = pos;
  cmd_velocity_ = vel;
  cmd_kp_ = kp;
  cmd_kd_ = kd;
  cmd_effort_ = effort;
}

void CanDmActuator::updateFrequency(const ros::Time& stamp)
{
  try {
    frequency_ = 1.0 / (stamp - last_timestamp_).toSec();
  } catch (const std::runtime_error& ex) {
    frequency_ = 0.0;
  }
}

void CanDmActuator::delayMicroseconds(unsigned int us)
{
  // 使用C++11的标准库实现微秒级延时
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} // namespace device
