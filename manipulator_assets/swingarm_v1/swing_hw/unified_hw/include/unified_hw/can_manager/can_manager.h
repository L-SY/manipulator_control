//
// Created by lsy on 24-12-16.
//

#pragma once

#include "can_devices/can_device.h"
#include "unified_hw/can_manager/can_devices/can_dm_actuator.h"
#include "unified_hw/can_manager/can_devices/can_rm_actuator.h"
#include "unified_hw/can_manager/can_interface/can_bus.h"
#include <ros/ros.h>

#include <memory>
#include <unordered_map>
#include <vector>
#include <thread>
#include <mutex>

namespace device {

class CanManager {
public:
  explicit CanManager(ros::NodeHandle& can_device_nh);
  ~CanManager();

  bool init();

  bool addCanBus(const std::string& bus_name, int thread_priority);

  bool addDevice(const std::string& name, const std::string& bus, int id, const std::string& model,
                 const XmlRpc::XmlRpcValue& config = XmlRpc::XmlRpcValue());

  bool start();

  void close();

  void read();

  void write();

  const std::unordered_map<std::string, std::shared_ptr<CanDevice>>&getDevices() const {
    return devices_;
  }

  const std::vector<std::string>& getActuatorNames() const {
    return actuator_names_;
  }

  const std::unordered_map<std::string, std::shared_ptr<CanActuator>>& getActuatorDevices() const {
    return actuator_devices_;
  }

  void delayMicroseconds(unsigned int us) {
    // Using C++11 standard library to achieve microsecond delay
    std::this_thread::sleep_for(std::chrono::microseconds(us));
  }
private:
  std::vector<can_interface::CanBus*>  can_buses_{};

  std::unordered_map<std::string, std::shared_ptr<CanDevice>> devices_;

  std::vector<std::string> actuator_names_;
  std::unordered_map<std::string, std::shared_ptr<CanActuator>> actuator_devices_;

  std::unordered_map<std::string, std::unordered_map<int, std::shared_ptr<CanDevice>>> bus_devices_;

  bool running_;
  std::vector<std::thread> read_threads_;
  std::mutex devices_mutex_;

  ros::NodeHandle nh_;

  bool loadBusConfig();
  bool loadDeviceConfig();
};

} // namespace device

