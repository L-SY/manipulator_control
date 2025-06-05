// can_dm_actuator.cpp

#include "unified_hw/can_manager/can_devices/can_rm_actuator.h"

namespace device {

CanRmActuator::CanRmActuator(const std::string &name, const std::string &bus,
                             int id, const std::string &motor_type,
                             const XmlRpc::XmlRpcValue &config)
    : CanActuator(name, bus, id, motor_type, DeviceType::read_write, config) {
  coeff_ = getCoefficientsFor();
  if (config.hasMember("max_out")) {
    max_out_ = static_cast<double>(config["max_out"]);
  } else {
    throw std::invalid_argument("Missing max_out in config");
  }
}

can_frame CanRmActuator::start() { return can_frame{}; }

can_frame CanRmActuator::close() { return can_frame{}; }

void CanRmActuator::read(const can_interface::CanFrameStamp &frameStamp) {
  const can_frame &frame = frameStamp.frame;

  ros::Time current_time = frameStamp.stamp;
  updateFrequency(current_time);
  last_timestamp_ = current_time;
  seq_++;
}

void CanRmActuator::readBuffer(
    const std::vector<can_interface::CanFrameStamp> &frameStamps) {
  for (const auto &frameStamp : frameStamps) {
    if (frameStamp.frame.can_id == master_id_) {
      if ((frameStamp.frame.data[0] & 0x0F) == static_cast<uint8_t>(id_)) {
        read(frameStamp);
        break;
      }
    }
  }
}

can_frame CanRmActuator::write() {
  can_frame frame{};

  return frame;
}

ActuatorCoefficients CanRmActuator::getCoefficientsFor() const {
  if (model_ == "rm_3508") {
    return ActuatorCoefficients(0.0007669903, 0.1047197551, 1.90702994e-5, 0.0,
                                0.0, 0.0, 0.0, 52437.561519, 0.0, 0.0, 0.0);
  } else if (model_ == "rm_6020") {
    return ActuatorCoefficients(0.0007670840, 0.1047197551, 5.880969e-5, 0.0,
                                0.0, 0.0, 0.0, 25000.0, 0.0, 0.0, 0.0);
  } else if (model_ == "rm_2006") {
    return ActuatorCoefficients(2.13078897e-5, 0.0029088820, 0.00018, 0.0, 0.0,
                                0.0, 0.0, 5555.5555555, 0.0, 0.0, 0.0);
  } else {
    throw std::invalid_argument("CanRmActuator: unsupported motor_type → " +
                                model_);
  }
}
} // namespace device