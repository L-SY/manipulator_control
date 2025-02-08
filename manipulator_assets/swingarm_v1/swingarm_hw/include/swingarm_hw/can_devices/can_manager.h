//
// Created by lsy on 24-12-16.
//

#pragma once

#include <ros/ros.h>
#include "can_device.h"
#include "swingarm_hw/can_interface/can_bus.h"
#include "swingarm_hw/can_devices/can_dm_actuator.h"
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

  bool addDevice(const std::string& name, const std::string& bus, int id, const std::string& model);

  std::shared_ptr<CanDevice> getDevice(const std::string& device_name);

  bool start();

  void close();

  void read();

  void write();

  std::unordered_map<std::string, std::shared_ptr<CanDevice>> getDevices(){return devices_;}

  std::unordered_map<std::string, std::shared_ptr<CanDmActuator>> getActuatorDevices(){return actuator_devices_;}
private:
  std::vector<can_interface::CanBus*>  can_buses_{};

  std::unordered_map<std::string, std::shared_ptr<CanDevice>> devices_;

  std::unordered_map<std::string, std::shared_ptr<CanDmActuator>> actuator_devices_;

  std::unordered_map<std::string, std::unordered_map<int, std::shared_ptr<CanDevice>>> bus_devices_;

  bool running_;
  std::vector<std::thread> read_threads_;
  std::mutex devices_mutex_;

  ros::NodeHandle nh_;

  bool loadBusConfig();
  bool loadDeviceConfig();
};

} // namespace device

