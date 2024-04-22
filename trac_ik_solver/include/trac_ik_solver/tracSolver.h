#pragma once

#include <vector>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include "ros/ros.h"

namespace trac_ik
{

class TracIKSolver
{
public:
  TracIKSolver(ros::NodeHandle nh);
  ~TracIKSolver();

  std::vector<double> computeFKReturnEulerAngles(std::vector<double> joint_values);
  std::vector<double> computeFKReturnQuaternion(std::vector<double> joint_values);
  std::vector<double> computeIKWithEulerAngles(const std::vector<double>& current_joint_positions,
                                               const std::vector<double>& target_pose_euler);
  std::vector<double> computeIKWithQuaternion(const std::vector<double>& current_joint_positions,
                                              const std::vector<double>& target_pose_quaternion);

private:
  void initSolver(const std::string& chain_start, const std::string& chain_end, const std::string& urdf_param,
                  double timeout, double eps);
  KDL::Chain chain_;
  KDL::JntArray ll_;  // Lower joint limits
  KDL::JntArray ul_;  // Upper joint limits
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_;
};

}  // namespace trac_ik
