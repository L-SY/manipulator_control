//
// Created by lsy on 24-12-16.
//

#pragma once

#include "swingarm_hw/can_interface/socketcan.h"

namespace device {
class CanDevice {
public:
  CanDevice(const std::string& name, const std::string& bus, int id, const std::string& type)
      : name_(name)
        , bus_(bus)
        , id_(id)
        , type_(type)
        , is_halted_(false)
  {
  }

  virtual ~CanDevice() = default;

  virtual void read(const can_frame &frame) = 0;

  virtual can_frame write() = 0;

  std::string getName() const { return name_; }
  std::string getBus() const { return bus_; }
  int getId() const { return id_; }
  std::string getType() const { return type_; }

  bool isValid() const { return is_halted_; }
  void setValid(bool valid) { is_halted_ = valid; }

protected:
  std::string name_;
  std::string bus_;
  int id_;
  std::string type_;
  bool is_halted_;
};
} // namespace device