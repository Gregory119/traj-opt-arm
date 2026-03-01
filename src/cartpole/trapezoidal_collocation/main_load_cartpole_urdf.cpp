#include <iostream>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/parsers/urdf.hpp"

int main(int argc, char **argv)
{
    namespace pin = pinocchio;

    if (argc != 2) {
        std::cout << "Path to model required." << std::endl;
        return 0;
    }

    // Load the urdf model
    const std::string urdf_filename = argv[1];
    pin::Model model;
    pin::urdf::buildModel(urdf_filename, model);
    std::cout << "model name: " << model.name << std::endl;

    // Create data required by the algorithms
    pin::Data data(model);

    Eigen::VectorXd q = pinocchio::neutral(model);
    q(2) = 3.142 / 2;  // Set the angle of the pole to pi/2 which should produce
                       // a positive torque
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    std::cout << "q: " << q.transpose() << std::endl;

    // Perform the forward kinematics over the kinematic tree
    forwardKinematics(model, data, q);

    // Print out the placement of each joint of the kinematic tree
    for (pin::JointIndex joint_id = 0;
         joint_id < ( pin::JointIndex )model.njoints;
         ++joint_id)
        std::cout << std::setw(24) << std::left << model.names[joint_id] << ": "
                  << std::fixed << std::setprecision(2)
                  << data.oMi[joint_id].translation().transpose() << std::endl;

    // calculate joint torques using inverse dynamics
    const Eigen::VectorXd &tau = pin::rnea(model, data, q, v, a);
    std::cout << "tau = " << tau.transpose() << std::endl;

    // calculate the partial derivative of generalized joint acceleration w.r.t
    // the generalized joint configuration, joint velocity, and joint torque
    pin::computeABADerivatives(model, data, q, v, tau);
    std::cout << std::endl << std::endl;
    std::cout << "Jacobians of forward dynamics:" << std::endl;
    std::cout << "da_dq = \n" << data.ddq_dq << std::endl;
    std::cout << "da_dv = \n" << data.ddq_dv << std::endl;
    std::cout << "da_dtau = \n" << data.ddq_dtau << std::endl;
}
