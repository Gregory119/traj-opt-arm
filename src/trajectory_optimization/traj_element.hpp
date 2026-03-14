#pragma once
#include <Eigen/Dense>
#include <deque>
#include <vector>

struct JointState
{
    double time;
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
};

using DiscreteJointStateTraj = std::deque<JointState>;
