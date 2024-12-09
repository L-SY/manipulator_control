//
// Created by lsy on 24-4-12.
//
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include "ros/ros.h"

namespace trac_ik
{
class TracIKSolver
{
public:
  TracIKSolver(ros::NodeHandle nh)
  {
    std::string chain_start, chain_end, urdf_param;
    double timeout, eps;
    nh.param("timeout", timeout, 0.005);
    nh.param("eps", eps, 1e-5);
    nh.param("chain_start", chain_start, std::string(""));
    nh.param("chain_end", chain_end, std::string(""));
    if (chain_start == "" || chain_end == "")
    {
      ROS_FATAL("Missing chain info in launch file");
      exit(-1);
    }

    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));
    initSolver(chain_start, chain_end, urdf_param, timeout, eps);
  }

  std::vector<double> computeFKReturnEulerAngles(std::vector<double> joint_values)
  {
    if (joint_values.size() != chain_.getNrOfJoints())
    {
      std::cout << "Incorrect number of joint values provide" << std::endl;
    }
    KDL::JntArray joint_positions(chain_.getNrOfJoints());
    for (size_t i = 0; i < joint_values.size(); ++i)
    {
      joint_positions(i) = joint_values[i];
    }
    KDL::Frame end_effector_pose;
    if (fk_solver_->JntToCart(joint_positions, end_effector_pose) < 0)
    {
      std::cout << ("Forward kinematics computation failed.") << std::endl;
    }

    std::vector<double> pose(6);
    double roll, pitch, yaw;
    end_effector_pose.M.GetRPY(roll, pitch, yaw);
    pose[0] = end_effector_pose.p.x();
    pose[1] = end_effector_pose.p.y();
    pose[2] = end_effector_pose.p.z();
    pose[3] = roll;
    pose[4] = pitch;
    pose[5] = yaw;

    return pose;
  }

  std::vector<double> computeFKReturnQuaternion(std::vector<double> joint_values)
  {
    if (joint_values.size() != chain_.getNrOfJoints())
    {
      std::cout << "Incorrect number of joint values provide" << std::endl;
    }

    KDL::JntArray joint_positions(chain_.getNrOfJoints());
    for (size_t i = 0; i < joint_values.size(); ++i)
    {
      joint_positions(i) = joint_values[i];
    }

    KDL::Frame end_effector_pose;
    if (fk_solver_->JntToCart(joint_positions, end_effector_pose) < 0)
    {
      std::cout << ("Forward kinematics computation failed.") << std::endl;
    }

    std::vector<double> pose(7);
    KDL::Rotation rotation = end_effector_pose.M;
    double x, y, z, w;
    rotation.GetQuaternion(x, y, z, w);

    pose[0] = end_effector_pose.p.x();
    pose[1] = end_effector_pose.p.y();
    pose[2] = end_effector_pose.p.z();
    pose[3] = w;
    pose[4] = x;
    pose[5] = y;
    pose[6] = z;

    return pose;
  }

  std::vector<double> computeIKWithEulerAngles(const std::vector<double>& current_joint_positions,
                                               const std::vector<double>& target_pose_euler)
  {
    if (target_pose_euler.size() != 6)
    {
      std::cout << ("Incorrect size of target pose vector.") << std::endl;
    }

    KDL::JntArray current_joints(chain_.getNrOfJoints());
    for (size_t i = 0; i < current_joint_positions.size(); ++i)
    {
      current_joints(i) = current_joint_positions[i];
    }

    // Convert target pose from Euler to KDL::Frame
    double x = target_pose_euler[0];
    double y = target_pose_euler[1];
    double z = target_pose_euler[2];
    double roll = target_pose_euler[3];
    double pitch = target_pose_euler[4];
    double yaw = target_pose_euler[5];
    KDL::Frame target_frame(KDL::Rotation::RPY(roll, pitch, yaw), KDL::Vector(x, y, z));

    KDL::JntArray result_joints(chain_.getNrOfJoints());
    if (trac_ik_solver_->CartToJnt(current_joints, target_frame, result_joints) < 0)
    {
      std::cout << ("Inverse kinematics computation failed.") << std::endl;
    }

    std::vector<double> result(current_joint_positions.size());
    for (size_t i = 0; i < result.size(); ++i)
    {
      result[i] = result_joints(i);
    }
    return result;
  }

  std::vector<double> computeIKWithQuaternion(const std::vector<double>& current_joint_positions,
                                              const std::vector<double>& target_pose_quaternion)
  {
    if (target_pose_quaternion.size() != 7)
    {
      std::cout << ("Incorrect size of target pose vector.") << std::endl;
    }

    KDL::JntArray current_joints(chain_.getNrOfJoints());
    for (size_t i = 0; i < current_joint_positions.size(); ++i)
    {
      current_joints(i) = current_joint_positions[i];
    }

    // Convert target pose from quaternion to KDL::Frame
    double x = target_pose_quaternion[0];
    double y = target_pose_quaternion[1];
    double z = target_pose_quaternion[2];
    double qx = target_pose_quaternion[3];
    double qy = target_pose_quaternion[4];
    double qz = target_pose_quaternion[5];
    double qw = target_pose_quaternion[6];
    KDL::Frame target_frame(KDL::Rotation::Quaternion(qx, qy, qz, qw), KDL::Vector(x, y, z));

    KDL::JntArray result_joints(chain_.getNrOfJoints());
    if (trac_ik_solver_->CartToJnt(current_joints, target_frame, result_joints) < 0)
    {
      std::cout << ("Inverse kinematics computation failed.") << std::endl;
    }

    std::vector<double> result(current_joint_positions.size());
    for (size_t i = 0; i < result.size(); ++i)
    {
      result[i] = result_joints(i);
    }
    return result;
  }

  ~TracIKSolver() = default;

private:
  void initSolver(const std::string& chain_start, const std::string& chain_end, const std::string& urdf_param,
                  double timeout, double eps)
  {
    trac_ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(chain_start, chain_end, urdf_param, timeout, eps);

    if (!trac_ik_solver_->getKDLChain(chain_))
    {
      ROS_FATAL("Failed to get valid KDL chain");
      exit(-1);
    }

    if (!trac_ik_solver_->getKDLLimits(ll_, ul_))
    {
      ROS_FATAL("Failed to get valid KDL joint limits");
      exit(-1);
    }

    assert(chain_.getNrOfJoints() == ll_.data.size());
    assert(chain_.getNrOfJoints() == ul_.data.size());

    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  }

  KDL::Chain chain_;
  KDL::JntArray ll_;
  KDL::JntArray ul_;
  KDL::Frame end_effector_pose;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_;
};

}  // namespace trac_ik

int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "trac_solver");
  ros::NodeHandle nh("~");
  trac_ik::TracIKSolver ikSolver(nh);
  std::vector<double> res = ikSolver.computeFKReturnEulerAngles({0.,0.,0.,0.,0.,0.});
  for (size_t i = 0; i < res.size(); ++i)
  {
    ROS_INFO_STREAM("Value = " << res[i]);
  }
  return 0;
}