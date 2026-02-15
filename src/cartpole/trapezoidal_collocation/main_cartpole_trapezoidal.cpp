#include <iostream>
#include <numbers>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

//#include "control_effort_trapezoidal_cost.hpp"
#include "trajectory_state_variables.hpp"
//#include "trapezoidal_collocation_constraints.hpp"

ifopt::Component::VecBound createStateBounds(const int num_state_vars,
                                             const size_t state_len,
                                             const double d,
                                             const double d_max)
{
    // vector of bounds initally all zero
    ifopt::Component::VecBound bounds(num_state_vars);
    // for each state vector (path bounds)
    for (size_t i{}; i < bounds.size(); i += state_len) {
        if (i == 0) {
            // initial state bounds
            for (size_t j{}; j < state_len; ++j) {
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

int main(int /*argc*/, char ** /*argv*/)
{
    // define problem
    ifopt::Problem nlp;

    // final q0 position
    const double d = 0.8;

    // state bounds
    const double d_max = 2 * d;
    const size_t state_len = 4;
    const int num_segments = 100;
    const int num_state_vars = (num_segments + 1) * state_len;
    ifopt::Component::VecBound bounds
        = createStateBounds(num_state_vars, state_len, d, d_max);

    // init guess for state variables
    auto x_init_state = Eigen::VectorXd::Zero(num_state_vars);
    nlp.AddVariableSet(
        std::make_shared<TrajectoryStateVariables>(std::move(x_init_state),
                                                   std::move(bounds)));

    // todo: trajectory control variables

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
