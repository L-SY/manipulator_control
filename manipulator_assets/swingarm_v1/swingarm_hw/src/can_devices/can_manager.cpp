//
// Created by lsy on 24-12-16.
//

#include "swingarm_hw/can_devices/can_manager.h"

namespace device {

CanManager::CanManager(ros::NodeHandle& can_device_nh)
    : nh_(can_device_nh), running_(false) {
}

bool CanManager::init() {
  if (!loadBusConfig()) {
    ROS_ERROR("Failed to load CAN bus configuration");
    return false;
  }

  if (!loadActuatorConfig()) {
    ROS_ERROR("Failed to load actuator configuration");
    return false;
  }

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

bool CanManager::loadActuatorConfig() {
  XmlRpc::XmlRpcValue actuators_config;
  if (!nh_.getParam("actuators", actuators_config)) {
    ROS_ERROR("No actuator configuration found");
    return false;
  }

  return parseActuators(actuators_config);
}

bool CanManager::parseActuators(XmlRpc::XmlRpcValue& actuators_config) {
  if (actuators_config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("Actuators configuration should be a struct");
    return false;
  }

  for (auto it = actuators_config.begin(); it != actuators_config.end(); ++it) {
    const std::string& name = it->first;
    XmlRpc::XmlRpcValue& actuator = it->second;

    if (!actuator.hasMember("bus") || !actuator.hasMember("id") || !actuator.hasMember("type")) {
      ROS_ERROR_STREAM("Actuator " << name << " missing required parameters");
      continue;
    }

    std::string bus_name = static_cast<std::string>(actuator["bus"]);
    int id = static_cast<int>(actuator["id"]);
    std::string type = static_cast<std::string>(actuator["type"]);

    std::shared_ptr<CanDevice> device;
    if (type.find("DM") != std::string::npos) {
        device = std::make_shared<CanDmActuator>(name, bus_name, id, type);
    } else {
      ROS_ERROR_STREAM("Unknown device type: " << type);
      continue;
    }

    if (device && addDevice(device)) {
      ROS_INFO_STREAM("Added actuator: " << name << " (type: " << type << ", bus: " << bus_name << ", id: 0x" << std::hex << id << ")");
      auto motor = std::dynamic_pointer_cast<CanDmActuator>(device);
      actuator_devices_[name] = motor;
    } else {
      ROS_ERROR_STREAM("Failed to add actuator: " << name);
      return false;
    }
  }

  return true;
}
CanManager::~CanManager() {}

bool CanManager::addCanBus(const std::string& bus_name, int thread_priority) {
  can_buses_.push_back(new can_interface::CanBus(bus_name, thread_priority));
  bus_devices_[bus_name] = std::unordered_map<int, std::shared_ptr<CanDevice>>();
  return true;
}

bool CanManager::addDevice(std::shared_ptr<CanDevice> device) {
  std::lock_guard<std::mutex> lock(devices_mutex_);

  if (!device) return false;

  const std::string& name = device->getName();
  const std::string& bus_name = device->getBus();
  int device_id = device->getId();

  bool bus_found = false;
  for (const auto& bus : can_buses_) {
    if (bus && bus->getName() == bus_name) {
      bus_found = true;
      break;
    }
  }

  if (!bus_found) {
    ROS_ERROR_STREAM("Bus " << bus_name << " not found");
    return false;
  }

  devices_[name] = device;

  bus_devices_[bus_name][device_id] = device;

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

    can_bus->read(now);

    const auto& read_buffer = can_bus->getReadBuffer();
    const std::string& bus_name = can_bus->getName();

    for (const auto& frame_stamp : read_buffer) {
      auto bus_it = bus_devices_.find(bus_name);
      if (bus_it != bus_devices_.end()) {
        auto device_it = bus_it->second.find(frame_stamp.frame.can_id);
        if (device_it != bus_it->second.end()) {
          device_it->second->read(frame_stamp.frame);
        }
      }
    }
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
        can_frame frame = device_pair.second->write();
        can_bus->write(&frame);
      }
    }
  }
}

} // namespace device