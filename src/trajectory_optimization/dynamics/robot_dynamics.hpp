#pragma once

#include <Eigen/Dense>

#include "pinocchio/algorithm/aba-derivatives.hpp"

using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

/*
 * Calculate the output of the dynamics function given the current state, and
 * control input.
 */
Eigen::VectorXd dyn(const Eigen::VectorXd &state,
                    const Eigen::VectorXd &control,
                    const double /*time*/,
                    const pinocchio::Model &model);

/*
 * Calculate the jacobian of the robot dynamics function w.r.t the state input.
 */
Jacobian jacDynWrtState(const Eigen::VectorXd &state,
                        const Eigen::VectorXd &control,
                        const double /*time*/,
                        const pinocchio::Model &model);

/*
 * Calculate the jacobian of the cartpole dynamics function w.r.t the control
 * input.
 */
Jacobian jacDynWrtControl(const Eigen::VectorXd &state,
                          const Eigen::VectorXd &control,
                          const double /*time*/,
                          const pinocchio::Model &model);
