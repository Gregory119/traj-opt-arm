#include <iostream>
#include <numbers>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include "control_effort_trapezoidal_cost.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "trajectory_variables.hpp"
#include "trapezoidal_collocation_constraints.hpp"

namespace pin = pinocchio;

/*
 * Create an upper and lower bound for each state vector along the trajectory.
 * @param d Final position of the cart.
 * @param d_max Maximum position of the cart at any time along the trajectory.
 */
ifopt::Component::VecBound createStateBounds(const int num_state_vars,
                                             const int state_len,
                                             const double d_max,
                                             const Eigen::VectorXd &state_start,
                                             const Eigen::VectorXd &state_end)
{
    // vector of bounds initally all zero
    ifopt::Component::VecBound bounds(num_state_vars);
    // for each state vector
    const int num_state_vecs = num_state_vars / state_len;
    for (size_t i{}; i < num_state_vecs; ++i) {
        if (i == 0) {
            // initial state bounds
            for (int j{}; j < state_len; ++j) {
                bounds[i * state_len + j] = {state_start(j), state_start(j)};
            }
        } else if (i == num_state_vecs - 1) {
            // final state bounds
            for (int j{}; j < state_len; ++j) {
                bounds[i * state_len + j] = {state_end(j), state_end(j)};
            }
        } else {
            // path bounds

            // bound for q0
            bounds[i * state_len] = {-d_max, d_max};
            // bound for q1
            bounds[i * state_len + 1]
                = {-2 * std::numbers::pi, 2 * std::numbers::pi};
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
    ifopt::Component::VecBound bounds(num_control_vars,
                                      {-max_force, max_force});
    return bounds;
}

/*
 * Calculate the output of the cartpole dynamics function given the current
 * state, and control input.
 */
Eigen::VectorXd cartpoleDyn(const Eigen::VectorXd &state,
                            const Eigen::VectorXd &control,
                            const double /*time*/,
                            const pin::Model &model)
{
    // data required by algorithm
    pin::Data data(model);

    // state = x = [q, dq] = [q1, q2, dq1, dq2]
    assert(state.size() == 4);
    // control = [u]
    assert(control.size() == 1);

    // Get the torque. The second joint torque is always zero (underactuated)
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    tau(0) = control(0);

    // Get the joint configuration.
    const auto q = state(Eigen::seqN(0, state.size() / 2));
    // Get the generalized joint velocity.
    const auto v = state(Eigen::seqN(state.size() / 2, state.size() / 2));

    // calculate the forward dynamics = [dq, ddq]
    pin::aba(model, data, q, v, tau);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(2 * model.nv);
    dx << v, data.ddq;  // concatenate
    // std::cout << "state = \n" << state << std::endl;
    // std::cout << "control = \n" << control << std::endl;
    // std::cout << "tau = \n" << tau << std::endl;
    // std::cout << "q = \n" << q << std::endl;
    // std::cout << "v = \n" << v << std::endl;
    // std::cout << "data.ddq = \n" << data.ddq << std::endl;
    return dx;
}

/*
 * Calculate the jacobian of the cartpole dynamics function w.r.t the state
 * input.
 */
ifopt::Component::Jacobian jacCartpoleDynWrtState(
    const Eigen::VectorXd &state,
    const Eigen::VectorXd &control,
    const double /*time*/,
    const pin::Model &model)
{
    // data required by algorithm
    pin::Data data(model);

    // state = x = [q, dq] = [q1, q2, dq1, dq2]
    assert(state.size() == 4);
    // control = [u]
    assert(control.size() == 1);

    // Get the torque. The second joint torque is always zero (underactuated)
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    tau(0) = control(0);

    // Get the joint configuration. Note that pinocchio has an additional
    // universe joint at index 0.
    const auto q = state(Eigen::seqN(0, state.size() / 2));
    // Get the generalized joint velocity.
    const auto v = state(Eigen::seqN(state.size() / 2, state.size() / 2));

    // calculate the partial derivative of generalized joint acceleration w.r.t
    // the generalized joint configuration, joint velocity, and joint torque
    Eigen::MatrixXd ddq_dq = Eigen::MatrixXd::Zero(model.nv, model.nv);
    Eigen::MatrixXd ddq_dv = Eigen::MatrixXd::Zero(model.nv, model.nv);
    Eigen::MatrixXd ddq_dtau = Eigen::MatrixXd::Zero(model.nv, model.nv);
    pin::computeABADerivatives(model,
                               data,
                               q,
                               v,
                               tau,
                               ddq_dq,
                               ddq_dv,
                               ddq_dtau);

    /*
      Jacobian of the forward dynamics function f w.r.t the state x=[q v] is:
      df/dx = [df/dq df/dv] =
      [dv/dq dv/dv;
       da/dq da/dv] =
      [0 I;
       da/dq da/dv]
      where v = dq/dt, a = dv / dt

      Note that the zero submatrix and identity submatrix each have
      size=(state_len/2 x state_len/2)
     */

    const int state_len = state.size();
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(state_len / 2 + state_len / 2 * state_len);
    // fill in dv/dv=I
    for (int i{}; i < state_len / 2; ++i) {
        for (int j{}; j < state_len / 2; ++j) {
            if (i == j) {
                triplets.push_back({i, j + state_len / 2, 1.0});
            }
        }
    }
    // fill in da/dq
    for (int i{}; i < ddq_dq.rows(); ++i) {
        for (int j{}; j < ddq_dq.cols(); ++j) {
            triplets.push_back({i + state_len / 2, j, ddq_dq(i, j)});
        }
    }
    // fill in da/dv
    for (int i{}; i < ddq_dv.rows(); ++i) {
        for (int j{}; j < ddq_dv.cols(); ++j) {
            triplets.push_back(
                {i + state_len / 2, j + state_len / 2, ddq_dv(i, j)});
        }
    }
    ifopt::Component::Jacobian jac(state_len, state_len);
    jac.setFromTriplets(triplets.cbegin(), triplets.cend());
    return jac;
}

/*
 * Calculate the jacobian of the cartpole dynamics function w.r.t the control
 * input.
 */

// todo: currently pin::computeABADerivatives() gets called twice for the same
// computation in jacCartpoleDynWrtState() and jacCartpoleDynWrtControl(). This
// is redundant because pin::computeABADerivatives() calculates the derivates
// required for the jacobian w.r.t the state and w.r.t the control. An
// optimization would be to refactor the code so that
// pin::computeABADerivatives() only gets called once between these two
// functions.
ifopt::Component::Jacobian jacCartpoleDynWrtControl(
    const Eigen::VectorXd &state,
    const Eigen::VectorXd &control,
    const double /*time*/,
    const pin::Model &model)
{
    // data required by algorithm
    pin::Data data(model);

    // state = x = [q, dq] = [q1, q2, dq1, dq2]
    assert(state.size() == 4);
    // control = [u]
    assert(control.size() == 1);

    // Get the torque. The second joint torque is always zero (underactuated)
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    tau(0) = control(0);

    // Get the joint configuration. Note that pinocchio has an additional
    // universe joint at index 0.
    const auto q = state(Eigen::seqN(0, state.size() / 2));
    // Get the generalized joint velocity.
    const auto v = state(Eigen::seqN(state.size() / 2, state.size() / 2));

    // calculate the partial derivative of generalized joint acceleration w.r.t
    // the generalized joint configuration, joint velocity, and joint torque
    Eigen::MatrixXd ddq_dq = Eigen::MatrixXd::Zero(model.nv, model.nv);
    Eigen::MatrixXd ddq_dv = Eigen::MatrixXd::Zero(model.nv, model.nv);
    Eigen::MatrixXd ddq_dtau = Eigen::MatrixXd::Zero(model.nv, model.nv);
    pin::computeABADerivatives(model,
                               data,
                               q,
                               v,
                               tau,
                               ddq_dq,
                               ddq_dv,
                               ddq_dtau);

    /*
      Jacobian of the forward dynamics function f w.r.t the control u=[tau(0)]
      is:
      df/du = df/dtau(0) =
      [dv/dtau(0);
       da/dtau(0)] =
      [0;
       da/dtau(0)]
      where v = dq/dt, a = dv / dt
     */

    const int state_len = state.size();
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(state_len / 2);
    // fill da/dtau(0), which is the first column of da/dtau, where tau is the
    // generalized joint vector.
    for (int i{}; i < state_len / 2; ++i) {
        triplets.push_back({i + state_len / 2, 0, ddq_dtau(i, 0)});
    }
    ifopt::Component::Jacobian jac(state_len, 1);
    jac.setFromTriplets(triplets.cbegin(), triplets.cend());
    return jac;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cout << "Path to model required." << std::endl;
        return 0;
    }

    // Load the urdf model
    const std::string urdf_filename = argv[1];
    pin::Model model;
    pin::urdf::buildModel(urdf_filename, model);
    std::cout << "model name: " << model.name << std::endl;

    // define problem
    ifopt::Problem nlp;
    const double traj_dur = 2.0;
    const int num_segments = 10;
    const double dt_segment = traj_dur / num_segments;

    // final q0 position
    const double d = 0.8;

    // state bounds
    const double d_max = 2 * d;
    const int state_len = 4;
    const int num_state_vars = (num_segments + 1) * state_len;
    const Eigen::VectorXd state_end{{d, std::numbers::pi, 0.0, 0.0}};
    // const Eigen::VectorXd state_start = state_end;
    const Eigen::VectorXd state_start = Eigen::VectorXd::Zero(state_len);
    ifopt::Component::VecBound state_bounds = createStateBounds(num_state_vars,
                                                                state_len,
                                                                d_max,
                                                                state_start,
                                                                state_end);

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
        return cartpoleDyn(state, control, time, model);
    };
    const auto jac_dyn_wrt_state_fn = [&](const Eigen::VectorXd &state,
                                          const Eigen::VectorXd &control,
                                          const double time) {
        return jacCartpoleDynWrtState(state, control, time, model);
    };
    const auto jac_dyn_wrt_control_fn = [&](const Eigen::VectorXd &state,
                                            const Eigen::VectorXd &control,
                                            const double time) {
        return jacCartpoleDynWrtControl(state, control, time, model);
    };
    const int num_constraints = state_len * num_segments;
    nlp.AddConstraintSet(std::make_shared<TrapezoidalCollocationConstraints>(
        num_constraints,
        traj_state_vars,
        state_len,
        traj_control_vars,
        control_len,
        dt_segment,
        dyn_fn,
        jac_dyn_wrt_state_fn,
        jac_dyn_wrt_control_fn));
    nlp.AddCostSet(std::make_shared<ControlEffortTrapezoidalCost>(
        "effort_cost",
        traj_control_vars->GetName(),
        control_len,
        dt_segment));
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
