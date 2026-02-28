#include <iostream>
#include <numbers>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

// #include "control_effort_trapezoidal_cost.hpp"
#include "trajectory_variables.hpp"
#include "trapezoidal_collocation_constraints.hpp"

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

struct CartpoleDynConsts
{
    constexpr double l = 0.5;
    constexpr double m1 = 2.0;
    constexpr double m2 = 0.5;
    constexpr double g = 9.81;
}

Eigen::VectorXd
    cartpoleDyn(const Eigen::VectorXd &state,
                const Eigen::VectorXd &control,
                const double /*time*/,
                const CartpoleDynConsts &c)
{
    // state = x = [q1, q2, dq1, dq2]
    assert(state.size() == 4);
    // control = [u]
    assert(control.size() == 1);

    auto dx = Eigen::VectorXd::Zero(4);
    // dx = [dq1, dq2, ddq1, ddq2]
    const auto q1 = state(0);
    const auto q2 = state(1);
    const auto dq1 = state(2);
    const auto dq2 = state(3);
    const auto u = control[0];
    dx(0) = dq1;
    dx(1) = dq2;
    dx(2) = (c.l * c.m2 * std::sin(q2) * std::pow(dq2, 2) + u
             + c.m2 * c.g * std::cos(q2) * std::sin(c.q2))
            / (c.m1 + c.m2 * (1.0 - std::pow(std::cos(q2), 2.0)));
    dx(3) = -(c.l * c.m2 * std::cos(q2) * std::sin(q2) * std::pow(dq2, 2)
              + u * std::cos(q2) + (c.m1 + c.m2) * c.g * std::sin(q2))
            / (c.l * c.m1 + c.l * c.m2 * (1.0 - std::pow(std::cos(q2), 2.0)));
    return dx;
}

int main(int /*argc*/, char ** /*argv*/)
{
    // define problem
    ifopt::Problem nlp;
    const double traj_dur = 2.0;
    const int num_segments = 100;
    const double dt_segment = traj_dur / num_segments;
    const CartpoleDynConsts dyn_consts{};

    // final q0 position
    const double d = 0.8;

    // state bounds
    const double d_max = 2 * d;
    const int state_len = 4;
    const int num_state_vars = (num_segments + 1) * state_len;
    ifopt::Component::VecBound state_bounds
        = createStateBounds(num_state_vars, state_len, d, d_max);

    // init guess for state variables
    auto state_init = Eigen::VectorXd::Zero(num_state_vars);
    auto traj_state_vars
        = std::make_shared<TrajectoryVariables>("traj_state_vars",
                                                std::move(state_init),
                                                std::move(state_bounds));
    nlp.AddVariableSet(traj_state_vars);

    // control bounds
    const int control_len = 1;
    const int num_control_vars = control_len * (num_segments + 1);
    const double max_control_force = 50;
    ifopt::Component::VecBound control_bounds
        = createControlBounds(num_control_vars, max_control_force);

    // init guess for control variables
    auto control_init = Eigen::VectorXd::Zero(num_control_vars);
    auto traj_control_vars
        = std::make_shared<TrajectoryVariables>("traj_control_vars",
                                                std::move(control_init),
                                                std::move(control_bounds));
    nlp.AddVariableSet(traj_control_vars);

    // add constraints
    const auto dyn_fn = [&](const Eigen::VectorXd &state,
                            const Eigen::VectorXd &control,
                            const double time) {
        cartpoleDyn(state, control, time, dyn_consts);
    };
    const int num_constraints = state_len * num_segments;
    nlp.AddConstraintSet(std::make_shared<TrapezoidalCollocationConstraints>(
        num_constraints,
        traj_state_vars->GetName(),
        state_len,
        traj_control_vars->GetName(),
        control_len,
        dt_segment,
        dyn_fn,
        // todo: jac_dyn_fn_wrt_state,
        // todo: jac_dyn_fn_wrt_control
        ));
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
