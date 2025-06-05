//
// Created by lsy on 25-6-4.
//

#include "unified_hw/can_manager/can_devices/can_rm_actuator.h"

namespace device {
CanRmActuator::CanRmActuator(const std::string &name, const std::string &bus,
                             int id, const std::string &motor_type,
                             const XmlRpc::XmlRpcValue &config)
    : CanDevice(name, bus, id, motor_type, DeviceType::read_write, config),
      coeff_(RmActuatorConfig().getRmActuatorConfig(motor_type)) {}

can_frame CanRmActuator::start() { return can_frame{}; }

can_frame CanRmActuator::close() { return can_frame{}; }

void CanRmActuator::read(const can_interface::CanFrameStamp &frameStamp) {
  auto frame = frameStamp.frame;
  uint16_t q_raw = (static_cast<uint16_t>(frame.data[0]) << 8u) | frame.data[1];
  uint16_t qd = (static_cast<uint16_t>(frame.data[2]) << 8u) | frame.data[3];
  uint16_t cur = (static_cast<uint16_t>(frame.data[4]) << 8u) | frame.data[5];

  //  if (act_data.seq != 0)  // not the first receive
  //  {
  //    if (act_data.q_raw - act_data.q_last > 4096)
  //      act_data.q_circle--;
  //    else if (act_data.q_raw - act_data.q_last < -4096)
  //      act_data.q_circle++;
  //  }

  position_ = coeff_.coeff.actuator2pos(q_raw);
  velocity_ = coeff_.coeff.actuator2vel(qd);
  effort_ = coeff_.coeff.actuator2effort(cur);

  ros::Time current_time = ros::Time::now();
  last_timestamp_ = current_time;
  seq_++;
}

void CanRmActuator::readBuffer(
    const std::vector<can_interface::CanFrameStamp> &frameStamps) {
  //  for (const auto& frameStamp : frameStamps) {
  //    if (frameStamp.frame.can_id == master_id_) {
  //      if ((frameStamp.frame.data[0] & 0b00001111) == id_) {
  //        read(frameStamp);
  //        break;
  //      }
  //    }
  //  }
}

can_frame CanRmActuator::write() {
  can_frame frame;

  return frame;
}

}