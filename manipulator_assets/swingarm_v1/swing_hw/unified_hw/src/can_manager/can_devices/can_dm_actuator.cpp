// can_dm_actuator.cpp

#include "unified_hw/can_manager/can_devices/can_dm_actuator.h"
#include <cstring>

namespace device {

CanDmActuator::CanDmActuator(const std::string& name,
                             const std::string& bus,
                             int id,
                             const std::string& motor_type,
                             const XmlRpc::XmlRpcValue& config)
    : CanActuator(name,
                  bus,
                  id,
                  motor_type,
                  DeviceType::read_write,
                  config)
{
  coeff_ = getCoefficientsFor();

  if (config.hasMember("max_velocity")) {
    max_velocity_ = static_cast<double>(config["max_velocity"]);
  } else {
    throw std::invalid_argument("Missing max_velocity in config");
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
  for (int i = 0; i < 7; ++i) {
    frame.data[i] = 0xFF;
  }
  frame.data[7] = 0xFD;
  ROS_WARN("Disabling motor %d", id_);
  return frame;
}

void CanDmActuator::read(const can_interface::CanFrameStamp& frameStamp) {
  const can_frame& frame = frameStamp.frame;
  uint16_t q_raw = (static_cast<uint16_t>(frame.data[1]) << 8) | static_cast<uint16_t>(frame.data[2]);
  uint16_t qd_raw = (static_cast<uint16_t>(frame.data[3]) << 4) | (static_cast<uint16_t>(frame.data[4]) >> 4);
  uint16_t cur = ((static_cast<uint16_t>(frame.data[4]) & 0xF) << 8) | static_cast<uint16_t>(frame.data[5]);

  // Multiple circle
  if (seq_ != 0)  // not the first receive
  {
    double new_position = coeff_.act2pos * static_cast<double>(q_raw) + coeff_.pos_offset + q_circle_ * 2 * M_PI;
    if (new_position - position_ > M_PI)
      q_circle_--;
    else if (new_position - position_ < -M_PI)
      q_circle_++;
  }

  position_ = coeff_.actuator2pos(q_raw) + q_circle_ * 2 * M_PI;
  velocity_ = coeff_.actuator2vel(qd_raw);
  effort_   = coeff_.actuator2effort(cur);

  ros::Time current_time = frameStamp.stamp;
  updateFrequency(current_time);
  last_timestamp_ = current_time;
  seq_++;
}

void CanDmActuator::readBuffer(const std::vector<can_interface::CanFrameStamp>& frameStamps) {
  for (const auto& frameStamp : frameStamps) {
    if (frameStamp.frame.can_id == master_id_) {
      if ((frameStamp.frame.data[0] & 0x0F) == static_cast<uint8_t>(id_)) {
        read(frameStamp);
        break;
      }
    }
  }
}

can_frame CanDmActuator::write() {
  can_frame frame{};
  if (control_mode_ == ControlMode::POSITION_VELOCITY) {
    frame = writePositionVelocity();
  } else {
    frame = writeEffortMIT();
  }
  return frame;
}

can_frame CanDmActuator::writeEffortMIT() {
  can_frame frame{};
  frame.can_id = id_;
  frame.can_dlc = 8;

  uint16_t q_des  = coeff_.pos2actuator(cmd_position_);
  uint16_t qd_des = coeff_.vel2actuator(cmd_velocity_);
  uint16_t kp     = coeff_.kp2actuator(cmd_kp_);
  uint16_t kd     = coeff_.kd2actuator(cmd_kd_);
  uint16_t tau    = coeff_.effort2actuator(cmd_effort_);

  frame.data[0] = static_cast<uint8_t>(q_des >> 8);
  frame.data[1] = static_cast<uint8_t>(q_des & 0xFF);
  frame.data[2] = static_cast<uint8_t>(qd_des >> 4);
  frame.data[3] = static_cast<uint8_t>(((qd_des & 0xF) << 4) | static_cast<uint8_t>(kp >> 8));
  frame.data[4] = static_cast<uint8_t>(kp & 0xFF);
  frame.data[5] = static_cast<uint8_t>(kd >> 4);
  frame.data[6] = static_cast<uint8_t>(((kd & 0xF) << 4) | static_cast<uint8_t>(tau >> 8));
  frame.data[7] = static_cast<uint8_t>(tau & 0xFF);

  return frame;
}

can_frame CanDmActuator::writePositionVelocity() {
  can_frame frame{};
  frame.can_id = id_ + 0x100;
  frame.can_dlc = 8;

  float p_des = static_cast<float>(cmd_position_);
  float v_des = static_cast<float>(max_velocity_);
  std::memcpy(&frame.data[0], &p_des, sizeof(float));
  std::memcpy(&frame.data[4], &v_des, sizeof(float));

  return frame;
}

ActuatorCoefficients CanDmActuator::getCoefficientsFor() const
{
  if (model_ == "DM4310") {
    return ActuatorCoefficients(
        0.00038147, 0.00510742, 0.0034188,
        8.19, 819.0, 2621.43812,
        195.793571, 292.500293,
        -12.5, -10.46, -7.0);
  }
  else if (model_ == "DM4340") {
    return ActuatorCoefficients(
        0.00038147, 0.00265625, 0.01318359,
        8.19, 819.0, 2621.43812,
        376.470588, 75.8518734,
        -12.5, -5.44, -27.0);
  }
  throw std::invalid_argument("Unsupported motor_type: " + model_);
}
} // namespace device