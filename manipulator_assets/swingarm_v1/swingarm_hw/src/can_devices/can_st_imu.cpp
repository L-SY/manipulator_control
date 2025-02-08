//
// Created by lsy on 25-2-8.
//

#include "swingarm_hw/can_devices/can_st_imu.h"
#include "ros/ros.h"

namespace device {

CanSTImu::CanSTImu(const std::string& name, const std::string& bus, int id)
    : CanDevice(name, bus, id, "st_imu", DeviceType::only_read),
      orientation_({0.0, 0.0, 0.0}),
      angular_velocity_({0.0, 0.0, 0.0}),
      linear_acceleration_({0.0, 0.0, 0.0}),
      temperature_(0.0)
{
  last_timestamp_ = ros::Time::now();
}

can_frame CanSTImu::start() {
  ROS_INFO_STREAM("Starting IMU device: " << getName());
  can_frame frame;
  return frame;
}

can_frame CanSTImu::close() {
  ROS_INFO_STREAM("Closing IMU device: " << getName());
  can_frame frame;
  return frame;
}

can_frame CanSTImu::write() {
  ROS_DEBUG_STREAM("Writing command to IMU device: " << getName());
  can_frame frame;
  return frame;
}

void CanSTImu::updateFrequency(const ros::Time& stamp) {
  double dt = (stamp - last_timestamp_).toSec();
  if (dt > 0)
    frequency_ = 1.0 / dt;
  else
    frequency_ = 0;
}

void CanSTImu::read(const can_frame& frame) {
  if(frame.can_id == getId()) {
    int16_t raw0 = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
    int16_t raw1 = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
    int16_t raw2 = static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]);

    angular_velocity_[0] = raw0 * angular_vel_coeff_ + angular_vel_offset_[0];
    angular_velocity_[1] = raw1 * angular_vel_coeff_ + angular_vel_offset_[1];
    angular_velocity_[2] = raw2 * angular_vel_coeff_ + angular_vel_offset_[2];

    int16_t temp_raw = static_cast<int16_t>((frame.data[6] << 3) | (frame.data[7] >> 5));
    if(temp_raw > 1023)
      temp_raw -= 2048;
    temperature_ = temp_raw * temp_coeff_ + temp_offset_;

    last_timestamp_ = ros::Time::now();
    updateFrequency(last_timestamp_);
    ROS_DEBUG_STREAM("IMU Gyro and temperature data updated for device: " << getName());
  }
  else if(frame.can_id == (getId() - 1)) {
    int16_t raw0 = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
    int16_t raw1 = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
    int16_t raw2 = static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]);

    linear_acceleration_[0] = raw0 * accel_coeff_;
    linear_acceleration_[1] = raw1 * accel_coeff_;
    linear_acceleration_[2] = raw2 * accel_coeff_;

    last_timestamp_ = ros::Time::now();
    updateFrequency(last_timestamp_);
    ROS_DEBUG_STREAM("IMU Accel data updated for device: " << getName());
  }
  else {
    ROS_WARN_STREAM("Received CAN frame with unexpected ID: " << frame.can_id
                                                              << " for IMU device: " << getName());
  }
}

void CanSTImu::readBuffer(const std::vector<can_interface::CanFrameStamp>& frames)
{
  for (const auto& frame : frames) {
    if (frame.frame.can_id == getId() || frame.frame.can_id == (getId() - 1)) {
      read(frame.frame);
    }
  }
}

} // namespace device