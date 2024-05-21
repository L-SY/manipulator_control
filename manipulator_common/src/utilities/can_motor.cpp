#include "arm_common/tools/can_motor.h"

namespace can_interface
{

bool CanMotor::init(XmlRpc::XmlRpcValue& act_coeffs, XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh)
{
  bool success = true;
  success &= parseActCoeffs(act_coeffs);
  success &= parseActData(act_datas, robot_hw_nh);
  success &= initCanBus(robot_hw_nh);

  return success;
};
bool CanMotor::parseActCoeffs(XmlRpc::XmlRpcValue& act_coeffs)
{
  ROS_ASSERT(act_coeffs.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try
  {
    for (auto it = act_coeffs.begin(); it != act_coeffs.end(); ++it)
    {
      can_interface::ActCoeff act_coeff{};

      act_coeff.act2pos = xmlRpcGetDouble(act_coeffs[it->first], "act2pos", 0.0001192087);
      act_coeff.act2vel = xmlRpcGetDouble(act_coeffs[it->first], "act2vel", 0.001022904);
      act_coeff.act2effort = xmlRpcGetDouble(act_coeffs[it->first], "act2effort", 0.00341797);

      act_coeff.pos2act = xmlRpcGetDouble(act_coeffs[it->first], "pos2act", 2621.4);
      act_coeff.vel2act = xmlRpcGetDouble(act_coeffs[it->first], "vel2act", 977.61267);
      act_coeff.effort2act = xmlRpcGetDouble(act_coeffs[it->first], "effort2act", 292.571322);

      act_coeff.pos_offset = xmlRpcGetDouble(act_coeffs[it->first], "pos_offset", -3.1415926);
      act_coeff.vel_offset = xmlRpcGetDouble(act_coeffs[it->first], "vel_offset", -20.943951);
      act_coeff.effort_offset = xmlRpcGetDouble(act_coeffs[it->first], "effort_offset", -7.0);
      act_coeff.kp2act = xmlRpcGetDouble(act_coeffs[it->first], "kp2act", 8.19);
      act_coeff.kd2act = xmlRpcGetDouble(act_coeffs[it->first], "kd2act", 819);

      act_coeff.max_out = xmlRpcGetDouble(act_coeffs[it->first], "max_out", 10000);
      std::string type = it->first;

      if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
        type2act_coeffs_.insert(std::make_pair(type, act_coeff));
      else
        ROS_ERROR_STREAM("Repeat actuator coefficient of type: " << type);
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                     << "configuration: " << e.getMessage() << ".\n"
                     << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}
bool CanMotor::parseActData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh)
{
  ROS_ASSERT(act_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try
  {
    for (auto it = act_datas.begin(); it != act_datas.end(); ++it)
    {
      if (!it->second.hasMember("bus"))
      {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated bus.");
        continue;
      }
      else if (!it->second.hasMember("type"))
      {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated type.");
        continue;
      }
      else if (!it->second.hasMember("id"))
      {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated ID.");
        continue;
      }
      double act_offset = it->second.hasMember("act_offset") ? ((double)it->second["act_offset"]) : 0.;
      std::string bus = act_datas[it->first]["bus"], type = act_datas[it->first]["type"];
      int id = static_cast<int>(act_datas[it->first]["id"]);
      // check define of act_coeffs
      if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
      {
        ROS_ERROR_STREAM("Type " << type << " has no associated coefficient.");
        return false;
      }
      // for bus interface
      if (bus_id2act_data_.find(bus) == bus_id2act_data_.end())
        bus_id2act_data_.insert(std::make_pair(bus, std::unordered_map<int, can_interface::ActData>()));

      if (!(bus_id2act_data_[bus].find(id) == bus_id2act_data_[bus].end()))
      {
        ROS_ERROR_STREAM("Repeat actuator on bus " << bus << " and ID " << id);
        return false;
      }
      else
      {
        ros::NodeHandle nh = ros::NodeHandle(robot_hw_nh, "actuators/" + it->first);
        bus_id2act_data_[bus].insert(std::make_pair(id, can_interface::ActData{ .name = it->first,
                                                                                .type = type,
                                                                                .stamp = ros::Time::now(),
                                                                                .seq = 0,
                                                                                .halted = false,
                                                                                .q_raw = 0,
                                                                                .qd_raw = 0,
                                                                                .temp = 0,
                                                                                .q_circle = 0,
                                                                                .q_last = 0,
                                                                                .frequency = 0,
                                                                                .pos = 0,
                                                                                .vel = 0,
                                                                                .effort = 0,
                                                                                .cmd_pos = 0,
                                                                                .cmd_vel = 0,
                                                                                .cmd_effort = 0,
                                                                                .exe_effort = 0,
                                                                                .act_offset = act_offset,
                                                                                .lp_filter = new LowPassFilter(nh) }));
      }
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                     << "configuration: " << e.getMessage() << ".\n"
                     << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}

bool CanMotor::initCanBus(ros::NodeHandle& robot_hw_nh)
{
  // CAN Bus
  XmlRpc::XmlRpcValue xml_rpc_value;
  int thread_priority_ = 95;
  if (!robot_hw_nh.getParam("bus", xml_rpc_value))
    ROS_WARN("No bus specified");
  else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ASSERT(xml_rpc_value[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    for (int i = 0; i < xml_rpc_value.size(); ++i)
    {
      std::string bus_name = xml_rpc_value[i];
      if (bus_name.find("can") != std::string::npos)
        can_buses_.push_back(
            new can_interface::CanBus(bus_name,
                                      can_interface::CanDataPtr{ .type2act_coeffs_ = &type2act_coeffs_,
                                                                 .id2act_data_ = &bus_id2act_data_[bus_name] },
                                      thread_priority_));
      else
        ROS_ERROR_STREAM("Unknown bus: " << bus_name);
    }
  }
  return true;
}

void CanMotor::startMotor()
{
  for (auto can_bus : can_buses_)
  {
    can_bus->start();
  }
}

void CanMotor::closeMotor()
{
  for (auto can_bus : can_buses_)
  {
    can_bus->close();
  }
}

void CanMotor::testMotor()
{
  for (auto can_bus : can_buses_)
  {
    can_bus->test();
  }
}
}  // namespace can_interface
