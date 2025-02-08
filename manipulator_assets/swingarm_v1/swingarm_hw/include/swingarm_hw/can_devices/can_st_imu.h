//
// Created by lsy on 25-2-8.
//

#pragma once

#include "can_device.h"
#include "ros/ros.h"
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

namespace device {

class CanSTImu : public CanDevice {
public:
  CanSTImu(const std::string& name, const std::string& bus, int id, std::string frame_id);
  ~CanSTImu() override = default;

  can_frame start() override;
  can_frame close() override;
  void read(const can_frame& frame) override;
  void readBuffer(const std::vector<can_interface::CanFrameStamp> &buffer) override;
  can_frame write() override;

  double getFrequency() const { return frequency_; }
  std::string getFrameID() const { return frame_id_; }
  const std::vector<double>& getAngularVelocity() const { return angular_velocity_; }
  const std::vector<double>& getLinearAcceleration() const { return linear_acceleration_; }
  const std::vector<double>& getOrientation() const { return orientation_; }
  const std::vector<double>& getOrientationCovariance() const { return orientation_covariance_diagonal_; }
  const std::vector<double>& getAngularVelocityCovariance() const { return angular_velocity_covariance_; }
  const std::vector<double>& getLinearAccelerationCovariance() const { return linear_acceleration_covariance_; }
  double getTemperature() const { return temperature_; }

private:
  std::string frame_id_;
  double frequency_{0};
  ros::Time last_timestamp_;
  uint32_t seq_{0};

  const std::vector<double> orientation_covariance_diagonal_{0.0012, 0.0012, 0.0012};
  const std::vector<double> angular_velocity_covariance_{0.0004, 0.0004, 0.0004};
  const std::vector<double> linear_acceleration_covariance_{0.01, 0.01, 0.01};
  const std::vector<double> angular_vel_offset_{-0.002433065, 0.003094675, 0.003542005};
  const double angular_vel_coeff_ = 0.0010652644;
  const double accel_coeff_ = 0.0017944335;
  const double temp_coeff_ = 0.125;
  const double temp_offset_ = 23.0;
  const std::string filter_ = "complementary";
  const bool do_bias_estimation_ = false;
  const bool do_adaptive_gain_ = true;
  const double gain_acc_ = 0.0003;

  std::vector<double> orientation_;         // [roll, pitch, yaw]
  std::vector<double> angular_velocity_;      // [x, y, z] 陀螺仪数据
  std::vector<double> linear_acceleration_;   // [x, y, z] 加速度数据
  double temperature_{0.0};

  void updateFrequency(const ros::Time& stamp);
};

} // namespace device