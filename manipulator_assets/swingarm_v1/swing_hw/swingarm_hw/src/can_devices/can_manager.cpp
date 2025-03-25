//
// Created by lsy on 24-12-16.
//

#include "swingarm_hw/can_devices/can_manager.h"

namespace device {

CanManager::CanManager(ros::NodeHandle& can_device_nh)
    : nh_(can_device_nh), running_(false) {
  init();
}

bool CanManager::init() {
  if (!loadBusConfig()) {
    ROS_ERROR("Failed to load CAN bus configuration");
    return false;
  }

  if (!loadDeviceConfig()) {
    ROS_ERROR("Failed to load CAN device configuration");
    return false;
  }

  start();
  return true;
}

bool CanManager::loadBusConfig() {
  XmlRpc::XmlRpcValue buses;
  if (!nh_.getParam("bus", buses)) {
    ROS_ERROR("No CAN bus configuration found");
    return false;
  }

  if (buses.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Bus parameter should be an array");
    return false;
  }

  for (int i = 0; i < buses.size(); ++i) {
    std::string bus_name = static_cast<std::string>(buses[i]);
    if (bus_name.find("can") != std::string::npos) {
      if (!addCanBus(bus_name, 95)) {
        ROS_ERROR_STREAM("Failed to add CAN bus: " << bus_name);
        return false;
      }
      ROS_INFO_STREAM("Added CAN bus: " << bus_name);
    }
  }

  return true;
}

bool CanManager::loadDeviceConfig() {
  XmlRpc::XmlRpcValue devices_param;
  if (!nh_.getParam("devices", devices_param)) {
    ROS_ERROR("No device configuration found");
    return false;
  }

  if (devices_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Device parameter should be an array");
    return false;
  }

  for (int i = 0; i < devices_param.size(); ++i) {
    if (devices_param[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_WARN_STREAM("Device configuration at index " << i << " is not a struct");
      continue;
    }

    if (!devices_param[i].hasMember("name") ||
        !devices_param[i].hasMember("bus") ||
        !devices_param[i].hasMember("id") ||
        !devices_param[i].hasMember("model")) {
      ROS_ERROR_STREAM("Device configuration at index " << i << " is missing required fields");
      return false;
    }

    std::string name  = static_cast<std::string>(devices_param[i]["name"]);
    std::string bus   = static_cast<std::string>(devices_param[i]["bus"]);
    int id            = static_cast<int>(devices_param[i]["id"]);
    std::string model = static_cast<std::string>(devices_param[i]["model"]);

    if (!addDevice(name, bus, id, model)) {
      ROS_ERROR_STREAM("Failed to add device: " << name);
      return false;
    }
    else
      ROS_INFO_STREAM("Add device: " << name);
  }

  return true;
}

CanManager::~CanManager() {}

bool CanManager::start() {
  std::lock_guard<std::mutex> lock(devices_mutex_);
  bool all_success = true;

  for (auto* can_bus : can_buses_) {
    if (!can_bus)
      continue;

    const std::string& bus_name = can_bus->getName();
    auto bus_it = bus_devices_.find(bus_name);

    if (bus_it != bus_devices_.end()) {
      for (const auto& device_pair : bus_it->second) {
        can_frame frame = device_pair.second->start();

        can_bus->write(&frame);
      }
    }
  }
  ROS_INFO_STREAM("All devices start right!");
  return all_success;
}

bool CanManager::addCanBus(const std::string& bus_name, int thread_priority) {
  can_buses_.push_back(new can_interface::CanBus(bus_name, thread_priority));
  bus_devices_[bus_name] = std::unordered_map<int, std::shared_ptr<CanDevice>>();
  return true;
}

bool CanManager::addDevice(const std::string& name,
                           const std::string& bus,
                           int id,
                           const std::string& model) {
  std::shared_ptr<CanDevice> device;
  if (model.find("dm") != std::string::npos) {
    device = std::make_shared<CanDmActuator>(name, bus, id, model);
    actuator_devices_[name] = std::dynamic_pointer_cast<CanDmActuator>(device);
    actuator_names_.push_back(name);
  }
  else if (model.find("st") != std::string::npos) {
    device = std::make_shared<CanSTImu>(name, bus, id, name.substr(0, name.size()-4));
    st_imu_devices_[name] = std::dynamic_pointer_cast<CanSTImu>(device);
    imu_names_.push_back(name);
  }
  else
    ROS_ERROR_STREAM("Unknown device model: " << model);

  if (!device) {
    ROS_ERROR_STREAM("Failed to create device: " << name);
    return false;
  }

  devices_[name] = device;

  bus_devices_[bus][id] = device;

  ROS_INFO_STREAM("Added device: " << name << " on bus: " << bus);
  return true;
}

std::shared_ptr<CanDevice> CanManager::getDevice(const std::string& device_name) {
  std::lock_guard<std::mutex> lock(devices_mutex_);
  auto it = devices_.find(device_name);
  return (it != devices_.end()) ? it->second : nullptr;
}

void CanManager::read() {
  std::lock_guard<std::mutex> lock(devices_mutex_);
  ros::Time now = ros::Time::now();

  for (auto* can_bus : can_buses_) {
    if (!can_bus) continue;

    const auto& read_buffer = can_bus->getReadBuffer();
    const std::string& bus_name = can_bus->getName();

    auto bus_it = bus_devices_.find(bus_name);
    if (bus_it == bus_devices_.end())
      continue;

    for (auto& device_pair : bus_it->second) {
      if (device_pair.second->getType() != DeviceType::only_write) {
        device_pair.second->readBuffer(read_buffer);
      }
    }

    can_bus->read(now);
  }
}

void CanManager::write() {
  std::lock_guard<std::mutex> lock(devices_mutex_);

  for (auto* can_bus : can_buses_) {
    if (!can_bus) continue;

    const std::string& bus_name = can_bus->getName();

    auto bus_it = bus_devices_.find(bus_name);
    if (bus_it != bus_devices_.end()) {
      for (const auto& device_pair : bus_it->second) {
        if (device_pair.second->getType() != DeviceType::only_read) {
          can_frame frame = device_pair.second->write();
          can_bus->write(&frame);
        }
      }
    }
  }
}

} // namespace device