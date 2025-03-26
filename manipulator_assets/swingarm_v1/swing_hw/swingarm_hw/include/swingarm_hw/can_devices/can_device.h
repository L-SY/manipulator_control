//
// Created by lsy on 24-12-16.
//

#pragma once

#include "swingarm_hw/can_interface/can_bus.h"
#include "vector"
#include <XmlRpcValue.h>

namespace device {

enum class DeviceType {
  only_read,
  only_write,
  read_write
};

class CanDevice {
public:
  CanDevice(const std::string& name, const std::string& bus, int id, const std::string& model, DeviceType type,
            const XmlRpc::XmlRpcValue& config = XmlRpc::XmlRpcValue())
      : name_(name)
        , bus_(bus)
        , id_(id)
        , model_(model)
        , type_(type)
        , is_halted_(false)
        , config_(config)
  {
  }

  CanDevice(const std::string& name, const std::string& bus, int id, const std::string& model, DeviceType type)
      : CanDevice(name, bus, id, model, type, XmlRpc::XmlRpcValue())
  {
  }

  virtual ~CanDevice() = default;

  virtual can_frame start() = 0;

  virtual can_frame close() = 0;

  virtual void read(const can_interface::CanFrameStamp& frameStamp) = 0;

  virtual void readBuffer(const std::vector<can_interface::CanFrameStamp> &buffer) = 0;

  virtual can_frame write() = 0;

  std::string getName() const { return name_; }
  std::string getBus() const { return bus_; }
  int getId() const { return id_; }

  DeviceType getType() const { return type_; }
  std::string getModel() const { return model_; }

  bool isValid() const { return is_halted_; }
  void setValid(bool valid) { is_halted_ = valid; }

protected:
  std::string name_;
  std::string bus_;
  int id_;
  DeviceType type_;
  std::string model_;
  bool is_halted_;
  XmlRpc::XmlRpcValue config_;
};

} // namespace device