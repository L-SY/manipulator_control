//
// Created by lsy on 25-2-8.
//

#include "swingarm_hw/can_devices/can_st_imu.h"
#include "ros/ros.h"

namespace device {

CanSTImu::CanSTImu(const std::string& name, const std::string& bus,const int id,const std::string frame_id, const XmlRpc::XmlRpcValue& config)
    : CanDevice(name, bus, id, "st_imu", DeviceType::only_read, config),
      orientation_({0.0, 0.0, 0.0}),
      angular_velocity_({0.0, 0.0, 0.0}),
      linear_acceleration_({0.0, 0.0, 0.0}),
      temperature_(0.0),
      frame_id_(frame_id)
{
  loadParameters(config);
  last_timestamp_ = ros::Time::now();
  imu_filter_ = new ImuComplementaryFilter;
  imu_filter_->init(config, name);
}

can_frame CanSTImu::start() {
  ROS_INFO_STREAM("Starting IMU device: " << getName());
  can_frame frame;
  for (int i = 0; i < 8; i++)
  {
    frame.data[i] = 0x00;
  }
  return frame;
}

can_frame CanSTImu::close() {
  ROS_INFO_STREAM("Closing IMU device: " << getName());
  can_frame frame;
  for (int i = 0; i < 8; i++)
  {
    frame.data[i] = 0x00;
  }
  return frame;
}

can_frame CanSTImu::write() {
  ROS_DEBUG_STREAM("Writing command to IMU device: " << getName());
  can_frame frame;
  for (int i = 0; i < 8; i++)
  {
    frame.data[i] = 0x00;
  }
  return frame;
}

void CanSTImu::updateFrequency(const ros::Time& stamp) {
  double dt = (stamp - last_timestamp_).toSec();
  if (dt > 0)
    frequency_ = 1.0 / dt;
  else
    frequency_ = 0;
}

void CanSTImu::read(const can_interface::CanFrameStamp& frameStamp) {
  auto frame = frameStamp.frame;
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

    imu_filter_->update(frameStamp.stamp, linear_acceleration_.data(), angular_velocity_.data(), orientation_.data(),
                                linear_acceleration_covariance_.data(), angular_velocity_covariance_.data(), orientation_covariance_diagonal_.data(),
                                temperature_, false);

    last_timestamp_ = ros::Time::now();
    updateFrequency(last_timestamp_);
    ROS_DEBUG_STREAM("IMU Accel data updated for device: " << getName());
  }
}

void CanSTImu::readBuffer(const std::vector<can_interface::CanFrameStamp>& frameStamps)
{
  for (const auto& frame : frameStamps) {
    if (frame.frame.can_id == getId() || frame.frame.can_id == (getId() - 1)) {
      read(frame);
    }
  }
}

void CanSTImu::loadParameters(const XmlRpc::XmlRpcValue& config) {
  // 读取向量参数的辅助函数
  auto readVector = [&config](const std::string& key, std::vector<double>& vec, const std::vector<double>& default_val) {
    if (config.hasMember(key) && config[key].getType() == XmlRpc::XmlRpcValue::TypeArray) {
      XmlRpc::XmlRpcValue array = config[key];
      if (array.size() == vec.size()) {
        for (int i = 0; i < array.size(); ++i) {
          if (array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
              array[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            vec[i] = static_cast<double>(array[i]);
          } else {
            ROS_WARN("Parameter %s[%d] is not a number, using default: %f",
                     key.c_str(), i, default_val[i]);
            vec[i] = default_val[i];
          }
        }
      } else {
        ROS_WARN("Parameter %s has wrong size %d (expected %zu), using defaults",
                 key.c_str(), array.size(), vec.size());
      }
    } else {
      ROS_INFO("Parameter %s not specified, using defaults", key.c_str());
    }
  };

  // 读取标量参数的辅助函数
  auto readScalar = [&config](const std::string& key, double& value, double default_val) {
    if (config.hasMember(key)) {
      if (config[key].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
          config[key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        value = static_cast<double>(config[key]);
      } else {
        ROS_WARN("Parameter %s is not a number, using default: %f", key.c_str(), default_val);
        value = default_val;
      }
    } else {
      ROS_INFO("Parameter %s not specified, using default: %f", key.c_str(), default_val);
    }
  };

  // 读取布尔参数的辅助函数
  auto readBool = [&config](const std::string& key, bool& value, bool default_val) {
    if (config.hasMember(key)) {
      if (config[key].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        value = static_cast<bool>(config[key]);
      } else {
        ROS_WARN("Parameter %s is not a boolean, using default: %s",
                 key.c_str(), default_val ? "true" : "false");
        value = default_val;
      }
    } else {
      ROS_INFO("Parameter %s not specified, using default: %s",
               key.c_str(), default_val ? "true" : "false");
    }
  };

  std::vector<double> default_orientation_cov = {0.0012, 0.0012, 0.0012};
  std::vector<double> default_angular_vel_cov = {0.0004, 0.0004, 0.0004};
  std::vector<double> default_linear_acc_cov = {0.01, 0.01, 0.01};
  std::vector<double> default_angular_vel_offset = {-0.002433065, 0.003094675, 0.003542005};

  readVector("orientation_covariance", orientation_covariance_diagonal_, default_orientation_cov);
  readVector("angular_velocity_covariance", angular_velocity_covariance_, default_angular_vel_cov);
  readVector("linear_acceleration_covariance", linear_acceleration_covariance_, default_linear_acc_cov);
  readVector("angular_velocity_offset", angular_vel_offset_, default_angular_vel_offset);

  readScalar("angular_velocity_coefficient", angular_vel_coeff_, 0.0010652644);
  readScalar("acceleration_coefficient", accel_coeff_, 0.0017944335);
  readScalar("temperature_coefficient", temp_coeff_, 0.125);
  readScalar("temperature_offset", temp_offset_, 23.0);
  readScalar("gain_acc", gain_acc_, 0.0003);

  readBool("do_bias_estimation", do_bias_estimation_, false);
  readBool("do_adaptive_gain", do_adaptive_gain_, true);

  ROS_INFO("IMU parameters loaded from configuration");
}

} // namespace device