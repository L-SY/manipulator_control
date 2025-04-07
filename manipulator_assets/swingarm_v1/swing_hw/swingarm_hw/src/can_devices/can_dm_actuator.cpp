//
// Created by lsy on 24-12-16.
//

// can_dm_actuator.cpp
#include "swingarm_hw/can_devices/can_dm_actuator.h"

namespace device {

CanDmActuator::CanDmActuator(const std::string& name, const std::string& bus, int id, const std::string& motor_type, const XmlRpc::XmlRpcValue& config)
    : CanDevice(name, bus, id, motor_type, DeviceType::read_write, config), coeff_(ActuatorConfig().getActuatorCoefficients(motor_type)) {
  if (config.hasMember("control_mode")) {
    std::string mode_str = static_cast<std::string>(config["control_mode"]);
    if (mode_str == "MIT") {
      control_mode_ = ControlMode::MIT;
    } else if (mode_str == "POSITION_VELOCITY") {
      control_mode_ = ControlMode::POSITION_VELOCITY;
    } else {
      control_mode_ = ControlMode::EFFORT;
    }
  }

  if (config.hasMember("master_id")) {
    master_id_ = static_cast<int>(config["master_id"]);
    if (config.hasMember("max_velocity")) {
      max_velocity_ = static_cast<double>(config["max_velocity"]);
    } else {
      throw std::invalid_argument("Missing max_velocity in config");
    }
  } else {
    throw std::invalid_argument("Missing master_id in config");
  }
}

can_frame CanDmActuator::createRegisterFrame(uint8_t command, uint8_t reg_addr, uint32_t value) const {
  can_frame frame{};
  frame.can_id = 0x7FF;

  if (command == CMD_READ || command == CMD_SAVE) {
    frame.can_dlc = 4;
    frame.data[0] = id_ & 0xFF;
    frame.data[1] = (id_ >> 8) & 0xFF;
    frame.data[2] = command;
    frame.data[3] = reg_addr;
  }
  else if (command == CMD_WRITE) {
    frame.can_dlc = 8;
    frame.data[0] = id_ & 0xFF;
    frame.data[1] = (id_ >> 8) & 0xFF;
    frame.data[2] = command;
    frame.data[3] = reg_addr;

    frame.data[4] = value & 0xFF;
    frame.data[5] = (value >> 8) & 0xFF;
    frame.data[6] = (value >> 16) & 0xFF;
    frame.data[7] = (value >> 24) & 0xFF;
  }

  return frame;
}

uint32_t CanDmActuator::controlModeToMotorMode(ControlMode mode) const {
  constexpr uint32_t MIT_MODE_VALUE = 1;
  constexpr uint32_t POS_VEL_MODE_VALUE = 2;
  constexpr uint32_t VEL_MODE_VALUE = 3;
  constexpr uint32_t POS_FORCE_MODE_VALUE = 4;
  constexpr uint32_t DEFAULT_MODE_VALUE = 1;

  switch (mode) {
  case ControlMode::MIT:
    return MIT_MODE_VALUE;

  case ControlMode::POSITION_VELOCITY:
    return POS_VEL_MODE_VALUE;

  case ControlMode::EFFORT:
    return MIT_MODE_VALUE;

  default:
    ROS_WARN("Unknown control mode requested, using default mode (MIT)");
    return DEFAULT_MODE_VALUE;
  }
}


can_frame CanDmActuator::start() {
  can_frame frame;

  if (start_call_count_ == 0) {
    uint32_t desiredMode = controlModeToMotorMode(control_mode_);
    frame = createRegisterFrame(CMD_WRITE, CTRL_MODE_REGISTER, desiredMode);
    ROS_WARN("Setting motor %d mode to %d", id_, desiredMode);
  }
  else if (start_call_count_ == 1) {
    frame = createRegisterFrame(CMD_SAVE, 0x00);
    ROS_WARN("Saving motor %d parameters", id_);
  }
  else {
    frame.can_id = id_;
    frame.can_dlc = 8;
    for (int i = 0; i < 7; i++) {
      frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFC;
    ROS_WARN("Enabling motor %d", id_);
  }

  start_call_count_ = start_call_count_ + 1;

  return frame;
}

can_frame CanDmActuator::close() {
  can_frame frame{};
  frame.can_id = id_;
  frame.can_dlc = 8;

  for (int i = 0; i < 7; i++) {
    frame.data[i] = 0xFF;
  }
  frame.data[7] = 0xFD;
  ROS_WARN("Disabling motor %d", id_);
  return frame;
}

void CanDmActuator::read(const can_interface::CanFrameStamp& frameStamp) {
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

void CanDmActuator::readBuffer(const std::vector<can_interface::CanFrameStamp>& frameStamps) {
  for (const auto& frameStamp : frameStamps) {
    if (frameStamp.frame.can_id == master_id_) {
      if ((frameStamp.frame.data[0] & 0b00001111) == id_) {
        read(frameStamp);
        break;
      }
    }
  }
}

can_frame CanDmActuator::write() {
  can_frame frame;

  if (control_mode_ == ControlMode::POSITION_VELOCITY) {
    frame = writePositionVelocity();
  } else {
    frame = writeEffortMIT();
  }

  delayMicroseconds(100);

  return frame;
}

can_frame CanDmActuator::writeEffortMIT() {
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

can_frame CanDmActuator::writePositionVelocity() {
  can_frame frame{};
  frame.can_id = id_ + 0x100;
  frame.can_dlc = 8;

  auto p_des = static_cast<float>(cmd_position_);
  auto v_des = static_cast<float>(max_velocity_);

  memcpy(&frame.data[0], &p_des, sizeof(float));
  memcpy(&frame.data[4], &v_des, sizeof(float));

  return frame;
}


void CanDmActuator::setCommand(double pos, double vel, double kp, double kd, double effort) {
  cmd_position_ = pos;
  cmd_velocity_ = vel;
  cmd_kp_ = kp;
  cmd_kd_ = kd;
  cmd_effort_ = effort;
}

void CanDmActuator::updateFrequency(const ros::Time& stamp) {
  try {
    frequency_ = 1.0 / (stamp - last_timestamp_).toSec();
  } catch (const std::runtime_error& ex) {
    frequency_ = 0.0;
  }
}

void CanDmActuator::delayMicroseconds(unsigned int us) {
  // Using C++11 standard library to achieve microsecond delay
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} // namespace device