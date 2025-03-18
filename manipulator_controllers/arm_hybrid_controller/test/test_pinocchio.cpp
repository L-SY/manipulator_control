//
// Created by lsy on 23-10-25.
//
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

int main(int argc, char** argv)
{
    pinocchio::Model model;
    pinocchio::buildModels::manipulator(model);
    pinocchio::Data data(model);

    Eigen::VectorXd q = pinocchio::neutral(model);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    const Eigen::VectorXd & tau = pinocchio::rnea(model,data,q,v,a);
    std::cout << "tau = " << tau.transpose() << std::endl;
}