//
// Created by lsy on 24-12-16.
//

#include "unified_hw/can_manager/can_manager.h"

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

    XmlRpc::XmlRpcValue config;
    if (devices_param[i].hasMember("config") &&
        devices_param[i]["config"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      config = devices_param[i]["config"];
    }

    if (!addDevice(name, bus, id, model, config)) {
      ROS_ERROR_STREAM("Failed to add device: " << name);
      return false;
    }
    else {
      ROS_INFO_STREAM("Add device: " << name);
    }
  }

  return true;
}

CanManager::~CanManager() {
  close();
}

bool CanManager::start() {
  std::lock_guard<std::mutex> lock(devices_mutex_);
  bool all_success = true;

  for (int i = 0; i < 5; i++) {
    ROS_INFO_STREAM("Starting devices, attempt " << (i+1) << " of 5");

    for (auto* can_bus : can_buses_) {
      if (!can_bus)
        continue;

      const std::string& bus_name = can_bus->getName();
      auto bus_it = bus_devices_.find(bus_name);

      if (bus_it != bus_devices_.end()) {
        for (const auto& device_pair : bus_it->second) {
          can_frame frame = device_pair.second->start();
          if (frame.can_dlc == 0) {
            continue;
          }
          delayMicroseconds(100000);
          can_bus->write(&frame);
        }
      }
    }
  }

  ROS_INFO_STREAM("All devices start command sent 5 times!");
  return all_success;
}

void CanManager::close() {
  std::lock_guard<std::mutex> lock(devices_mutex_);

  for (int i = 0; i < 5; i++) {
    for (auto* can_bus : can_buses_) {
      if (!can_bus)
        continue;

      const std::string& bus_name = can_bus->getName();
      auto bus_it = bus_devices_.find(bus_name);

      if (bus_it != bus_devices_.end()) {
        for (const auto& device_pair : bus_it->second) {
          can_frame frame = device_pair.second->close();
          if (frame.can_dlc == 0) {
            continue;
          }
          delayMicroseconds(100000);
          can_bus->write(&frame);
        }
      }
    }
  }
}


bool CanManager::addCanBus(const std::string& bus_name, int thread_priority) {
  can_buses_.push_back(new can_interface::CanBus(bus_name, thread_priority));
  bus_devices_[bus_name] = std::unordered_map<int, std::shared_ptr<CanDevice>>();
  return true;
}

bool CanManager::addDevice(const std::string& name,
                           const std::string& bus,
                           int id,
                           const std::string& model,
                           const XmlRpc::XmlRpcValue& config) {
  std::shared_ptr<CanActuator> device;

  if (model.find("DM") != std::string::npos) {
    device = std::make_shared<CanDmActuator>(name, bus, id, model, config);
    actuator_devices_[name] = device;
    actuator_names_.push_back(name);
  }
  else if (model.find("RM") != std::string::npos) {
    device = std::make_shared<CanRmActuator>(name, bus, id, model, config);
    actuator_devices_[name] = device;
    actuator_names_.push_back(name);
  }
  else {
    ROS_ERROR_STREAM("Unknown device model: " << model);
  }

  if (!device) {
    ROS_ERROR_STREAM("Failed to create device: " << name);
    return false;
  }

  devices_[name] = device;
  bus_devices_[bus][id] = device;

  ROS_INFO_STREAM("Added device: " << name << " on bus: " << bus);
  return true;
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
    if (bus_it == bus_devices_.end()) continue;
    can_frame rm_frame0_{}, rm_frame1_{};
    rm_frame0_.can_id  = 0x200;
    rm_frame0_.can_dlc = 8;
    rm_frame1_.can_id  = 0x1FF;
    rm_frame1_.can_dlc = 8;
    bool has_write_frame0 = false, has_write_frame1 = false;

    for (const auto& device_pair : bus_it->second) {
      auto device_ptr = device_pair.second;
      if (device_ptr->getType() == DeviceType::only_read) continue;

      can_frame frame = device_ptr->write();

      if (frame.can_id == 0x200) {
        for (int i = 0; i < 8; ++i) {
          if (frame.data[i] != 0) {
            rm_frame0_.data[i] = frame.data[i];
          }
          has_write_frame0 = true;
        }
      }
      else if (frame.can_id == 0x1FF) {
        for (int i = 0; i < 8; ++i) {
          if (frame.data[i] != 0) {
            rm_frame1_.data[i] = frame.data[i];
          }
          has_write_frame1 = true;
        }
      }
      else {
        can_bus->write(&frame);
      }
    }

    if (has_write_frame0) {
      can_bus->write(&rm_frame0_);
    }
    if (has_write_frame1) {
      can_bus->write(&rm_frame1_);
    }
  }
}


} // namespace device