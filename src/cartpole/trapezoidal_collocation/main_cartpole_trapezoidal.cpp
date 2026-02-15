#include <iostream>
#include <numbers>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include "control_effort_trapezoidal_cost.hpp"
#include "trajectory_state_variables.hpp"
#include "trapezoidal_collocation_constraints.hpp"

int main(int /*argc*/, char ** /*argv*/)
{
    // define problem
    ifopt::Problem nlp;

    // final q0 position
    const double d = 0.8;

    // state bounds
    const double d_max = 2 * d;
    const int state_len = 4;
    const auto state_bounds_fn
        = [&](const Eigen::VectorXd &x) -> ifopt::Component::VecBound {
        ifopt::Component::VecBound bound(x.size());
        // for each state vector (path bounds)
        for (int i{}; i < x.size(); i += state_len) {
            // each state element
            // bound for q0
            bound[i] = {-d_max, d_max};
            // bound for q1
            bound[i + 1] = {-2 * std::numbers::pi, 2 * std::numbers::pi};
        }

        // initial state
        const auto &x_first = x(segN(0, state_len));
        x_first.setZero();

        // final state
        const auto &x_last = x(Eigen::lastN(state_len));
        x_last[0] = d;
        x_last[1] = std::numbers::pi;
        x_last[2] = 0;
        x_last[3] = 0;

        return bound
    };
    nlp.AddVariableSet(
        std::make_shared<TrajectoryStateVariables>(100,        // num segments
                                                   state_len,  // state length
                                                   state_bounds_fn));

    // todo: trajectory control variables
    
    nlp.AddConstraintSet(std::make_shared<TrapezoidalCollocationConstraints>());
    nlp.AddCostSet(std::make_shared<ControlEffortTrapezoidalCost>());
    nlp.PrintCurrent();

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
