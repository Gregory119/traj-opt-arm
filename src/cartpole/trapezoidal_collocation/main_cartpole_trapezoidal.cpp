#include <iostream>
#include <numbers>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

// #include "control_effort_trapezoidal_cost.hpp"
#include "trajectory_variables.hpp"
// #include "trapezoidal_collocation_constraints.hpp"

ifopt::Component::VecBound createStateBounds(const int num_state_vars,
                                             const int state_len,
                                             const double d,
                                             const double d_max)
{
    // vector of bounds initally all zero
    ifopt::Component::VecBound bounds(num_state_vars);
    // for each state vector
    for (size_t i{}; i < bounds.size(); i += state_len) {
        if (i == 0) {
            // initial state bounds
            for (int j{}; j < state_len; ++j) {
                bounds[i + j] = {0.0, 0.0};
            }
        } else if (i == bounds.size() - 1) {
            // final state bounds
            bounds[i] = {d, d};
            bounds[i + 1] = {std::numbers::pi, std::numbers::pi};
            bounds[i + 2] = {0.0, 0.0};
            bounds[i + 3] = {0.0, 0.0};
        } else {
            // path bounds

            // bound for q0
            bounds[i] = {-d_max, d_max};
            // bound for q1
            bounds[i + 1] = {-2 * std::numbers::pi, 2 * std::numbers::pi};
            // bound for dq0
            bounds[i + 2] = {-ifopt::inf, ifopt::inf};
            // bound for dq1
            bounds[i + 3] = {-ifopt::inf, ifopt::inf};
        }
    }
    return bounds;
}

ifopt::Component::VecBound createControlBounds(const int num_control_vars,
                                               const double max_force)
{
    // vector of bounds all
    ifopt::Component::VecBound bounds(num_control_vars, {max_force, max_force});
    return bounds;
}

int main(int /*argc*/, char ** /*argv*/)
{
    // define problem
    ifopt::Problem nlp;

    // final q0 position
    const double d = 0.8;

    // state bounds
    const double d_max = 2 * d;
    const int state_len = 4;
    const int num_segments = 100;
    const int num_state_vars = (num_segments + 1) * state_len;
    ifopt::Component::VecBound state_bounds
        = createStateBounds(num_state_vars, state_len, d, d_max);

    // init guess for state variables
    auto state_init = Eigen::VectorXd::Zero(num_state_vars);
    nlp.AddVariableSet(
        std::make_shared<TrajectoryVariables>("traj_state_vars",
                                              std::move(state_init),
                                              std::move(state_bounds)));

    // control bounds
    const int control_len = 1;
    const int num_control_vars = control_len * (num_segments + 1);
    const double max_control_force = 50;
    ifopt::Component::VecBound control_bounds
        = createControlBounds(num_control_vars, max_control_force);

    // init guess for control variables
    auto control_init = Eigen::VectorXd::Zero(num_control_vars);
    nlp.AddVariableSet(
        std::make_shared<TrajectoryVariables>("traj_control_vars",
                                              std::move(control_init),
                                              std::move(control_bounds)));

    // nlp.AddConstraintSet(std::make_shared<TrapezoidalCollocationConstraints>());
    // nlp.AddCostSet(std::make_shared<ControlEffortTrapezoidalCost>());
    // nlp.PrintCurrent();

    // choose solver and options
    ifopt::IpoptSolver ipopt;
    ipopt.SetOption("tol", 3.82e-6);
    ipopt.SetOption("mu_strategy", "adaptive");
    ipopt.SetOption("output_file", "ipopt.out");

    // solve
    ipopt.Solve(nlp);
    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    std::cout << x.transpose() << std::endl;

    return 0;
}
