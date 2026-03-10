#pragma once
#include <Eigen/Dense>
#include <deque>
#include <vector>

struct TrajElement
{
    double time;
    Eigen::VectorXd val;
};

using SampleTraj = std::deque<TrajElement>;
