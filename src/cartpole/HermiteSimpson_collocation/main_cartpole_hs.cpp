#include <cubic_spline.hpp>
#include <iostream>
#include <numbers>
#include <quadratic_spline.hpp>
#include <traj_element.hpp>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>
#include <rapidcsv.h>

#include "control_effort_hs_cost.hpp"
#include "hermite_simpson_collocation_constraints.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "hs_traj_extractor.hpp"
#include "trajectory_variables.hpp"

#include "save_trajectory.hpp"


namespace pin = pinocchio;

/*
 * Create an upper and lower bound for each state vector along the trajectory.
 * @param d Final position of the cart.
 * @param d_max Maximum position of the cart at any time along the trajectory.
 */

ifopt::Component::VecBound createStateBounds(const int num_state_vars,
                                             const int state_len,
                                             double d_max,
                                             const Eigen::VectorXd &state_start,
                                             const Eigen::VectorXd &state_end)
{
    // vector of bounds initally all zero
    ifopt::Component::VecBound bounds;
    // for each state vector
    const int num_state_vecs = num_state_vars / state_len;
    for (int i{}; i < num_state_vecs; ++i) {
        if (i == 0) {
            // initial state bounds
            for (int j{}; j < state_len; ++j) {
                bounds.push_back({state_start(j), state_start(j)});
            }
        } else if (i == num_state_vecs - 1) {
            // final state bounds
            for (int j{}; j < state_len; ++j) {
                bounds.push_back({state_end(j), state_end(j)});
            }
        } else {
            // path bounds

            // bound for q0
            bounds.push_back({-d_max, d_max});
            // bound for q1
            bounds.push_back({-2 * std::numbers::pi, 2 * std::numbers::pi});
            // bound for dq0
            bounds.push_back({-ifopt::inf, ifopt::inf});
            // bound for dq1
            bounds.push_back({-ifopt::inf, ifopt::inf});
        }
    }
    assert(bounds.size() == num_state_vars);
    return bounds;
}

ifopt::Component::VecBound createMidpointStateBounds(const int num_state_vars,
                                                     const int state_len,
                                                     const double d_max)
{
    ifopt::Component::VecBound bounds;
    bounds.reserve(num_state_vars);
    for (int i = 0; i < num_state_vars; i += state_len) {
        // path bounds only
        // no pinned endpoints at midpoints
        bounds.push_back({-d_max, d_max});                                // q0
        bounds.push_back({-2 * std::numbers::pi, 2 * std::numbers::pi});  // q1
        bounds.push_back({-ifopt::inf, ifopt::inf});                      // dq0
        bounds.push_back({-ifopt::inf, ifopt::inf});                      // dq1
    }
    assert(bounds.size() == num_state_vars);
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

void guessStateTraj(const int state_len,
                    const int num_segments,
                    const Eigen::VectorXd &state_start,
                    const Eigen::VectorXd &state_end,
                    Eigen::VectorXd &state_col_init,
                    Eigen::VectorXd &state_mid_init)
{
    const int num_time_pts = num_segments + 1;
    // linearly interpolate from start state to end state
    for (int k{}; k < num_time_pts; ++k) {
        auto statek = state_col_init(Eigen::seqN(k * state_len, state_len));
        const double alpha
            = static_cast<double>(k)
              / (num_time_pts - 1);  // trajectory progress factor
        statek = alpha * (state_end - state_start) + state_start;

        // midpoint initialization
        if (k < num_segments) {
            auto state_mid_k
                = state_mid_init(Eigen::seqN(k * state_len, state_len));
            const double alpha_mid
                = (static_cast<double>(k) + 0.5) / num_segments;
             state_mid_k   = alpha_mid * (state_start - state_end) + state_start;
        }
    }
    return;
}

Eigen::VectorXd createStateGradTimeVars(
    const double start_time,
    const Eigen::VectorXd &traj_state_vars,
    const Eigen::VectorXd &traj_state_mid_vars,
    const int state_len,
    const Eigen::VectorXd &traj_control_vars,
    const Eigen::VectorXd &traj_control_mid_vars,
    const int control_len,
    const double dt_segment,
    const pin::Model &model)
{
    const int num_segments = traj_state_mid_vars.size() / state_len;
    Eigen::VectorXd traj_dstate_dt_vars
        = Eigen::VectorXd::Zero((2 * num_segments + 1) * state_len);

    for (int k = 0; k < num_segments; ++k) {
        const double tk = start_time + k * dt_segment;
        const double tc = tk + 0.5 * dt_segment;

        const Eigen::VectorXd xk
            = traj_state_vars(Eigen::seqN(k * state_len, state_len));
        const Eigen::VectorXd uk
            = traj_control_vars(Eigen::seqN(k * control_len, control_len));
        // center of segment
        const Eigen::VectorXd xc
            = traj_state_mid_vars(Eigen::seqN(k * state_len, state_len));
        const Eigen::VectorXd uc
            = traj_control_mid_vars(Eigen::seqN(k * control_len, control_len));
        // start point of segment derivative
        traj_dstate_dt_vars(Eigen::seqN((2 * k) * state_len, state_len))
            = cartpoleDyn(xk, uk, tk, model);
        // midpoint of segment derivative
        traj_dstate_dt_vars(Eigen::seqN((2 * k + 1) * state_len, state_len))
            = cartpoleDyn(xc, uc, tc, model);
    }

    const int N = num_segments;
    const double tN = start_time + N * dt_segment;
    const Eigen::VectorXd xN
        = traj_state_vars(Eigen::seqN(N * state_len, state_len));
    const Eigen::VectorXd uN
        = traj_control_vars(Eigen::seqN(N * control_len, control_len));
    // endpoint of segment derivative
    traj_dstate_dt_vars(Eigen::seqN((2 * N) * state_len, state_len))
        = cartpoleDyn(xN, uN, tN, model);

    return traj_dstate_dt_vars;
}

CubicSpline createStateSpline(const double start_time,
                              const double traj_dur,
                              const Eigen::VectorXd &traj_state_vars,
                              const int state_len,
                              const Eigen::VectorXd &traj_dstate_dt_vars)
{
    std::vector<Eigen::VectorXd> state_vals;
    std::vector<Eigen::VectorXd> state_grad_vals;

    const int num_state_vecs = traj_state_vars.size() / state_len;
    for (int i = 0; i < num_state_vecs; ++i) {
        const Eigen::VectorXd state
            = traj_state_vars(Eigen::seqN(i * state_len, state_len));

        // derivative vector
        const Eigen::VectorXd dstate_dt
            = traj_dstate_dt_vars(Eigen::seqN((2 * i) * state_len, state_len));

        state_vals.push_back(state);
        state_grad_vals.push_back(dstate_dt);
    }

    return CubicSpline(state_vals, state_grad_vals, start_time, traj_dur);
}

QuadraticSpline createStateGradTimeSpline(
    const double start_time,
    const double traj_dur,
    const Eigen::VectorXd &traj_dstate_dt_vars,
    const int state_len)
{
    std::vector<Eigen::VectorXd> knot_vals;
    std::vector<Eigen::VectorXd> midpoint_vals;

    const int num_blocks = traj_dstate_dt_vars.size() / state_len;
    const int num_segments = (num_blocks - 1) / 2;
    const int num_knots = num_segments + 1;

    for (int i = 0; i < num_knots; ++i) {
        knot_vals.push_back(
            traj_dstate_dt_vars(Eigen::seqN((2 * i) * state_len, state_len)));
    }

    for (int i = 0; i < num_segments; ++i) {
        midpoint_vals.push_back(traj_dstate_dt_vars(
            Eigen::seqN((2 * i + 1) * state_len, state_len)));
    }

    return QuadraticSpline(knot_vals, midpoint_vals, start_time, traj_dur);
}

// create sample trajectory
DiscreteJointStateTraj createSampleTraj(const double start_time,
                            const double sample_period,
                            const double traj_dur,
                            const CubicSpline &state_spline,
                            const QuadraticSpline &dstate_dt_spline)
{
    DiscreteJointStateTraj sample_traj;
    const int num_samples = static_cast<int>(traj_dur / sample_period) + 1;

    for (int i{}; i < num_samples; ++i) {
        const double time = i * sample_period + start_time;
        const Eigen::VectorXd state = state_spline.getValue(time);
        const Eigen::VectorXd dstate_dt = dstate_dt_spline.getValue(time);
        sample_traj.push_back(
            {.time = time,
             .q = state(Eigen::seqN(0, state.size() / 2)),
             .dq = state(Eigen::seqN(state.size() / 2, state.size() / 2)),
             .ddq
             = dstate_dt(Eigen::seqN(state.size() / 2, state.size() / 2))});
    }
    return sample_traj;
}

// save sample trajectory to file
void saveSampleTrajCsv(const std::string &filename,
                       const DiscreteJointStateTraj &sample_traj)
{
    // time | q(0) | q(1) | ... | q(n-1)
    rapidcsv::Document doc{};

    // create header
    doc.InsertColumn(0, std::vector<double>(), "time");
    std::array<std::string, 3> prefixes = {"q", "dq", "ddq"};
    for (int i{}; i < prefixes.size(); ++i) {
        for (int j{}; j < sample_traj[0].q.size(); ++j) {
            std::ostringstream os;
            os << prefixes[i] << j;
            doc.InsertColumn(i * sample_traj[0].q.size() + j + 1,
                             std::vector<double>(),
                             os.str());
        }
    }

    // fill in data in rows
    for (int i{}; i < sample_traj.size(); ++i) {
        std::vector<double> row_data;
        const JointState &e = sample_traj.at(i);
        row_data.push_back(e.time);
        row_data.insert(row_data.cend(), e.q.cbegin(), e.q.cend());
        row_data.insert(row_data.cend(), e.dq.cbegin(), e.dq.cend());
        row_data.insert(row_data.cend(), e.ddq.cbegin(), e.ddq.cend());
        doc.InsertRow(i, row_data);
    }
    doc.Save(filename);
}

DiscreteJointStateTraj createCollocationTraj(const double start_time,
                                 const double dt_segment,
                                 const Eigen::VectorXd &traj_state_vars,
                                 const Eigen::VectorXd &traj_state_mid_vars,
                                 const int state_len,
                                 const Eigen::VectorXd &traj_control_vars,
                                 const Eigen::VectorXd &traj_control_mid_vars,
                                 const int control_len,
                                 const pin::Model &model)
{
    DiscreteJointStateTraj traj;
    const int num_segments = traj_state_mid_vars.size() / state_len;
    // traj.reserve(2 * num_segments + 1);

    for (int k = 0; k < num_segments; ++k) {
        const double tk = start_time + k * dt_segment;
        const double tc = tk + 0.5 * dt_segment;

        const Eigen::VectorXd xk
            = traj_state_vars(Eigen::seqN(k * state_len, state_len));
        const Eigen::VectorXd uk
            = traj_control_vars(Eigen::seqN(k * control_len, control_len));
        const Eigen::VectorXd fk = cartpoleDyn(xk, uk, tk, model);

        traj.push_back({.time = tk,
                        .q = xk(Eigen::seqN(0, state_len / 2)),
                        .dq = xk(Eigen::seqN(state_len / 2, state_len / 2)),
                        .ddq = fk(Eigen::seqN(state_len / 2, state_len / 2))});

        const Eigen::VectorXd xc
            = traj_state_mid_vars(Eigen::seqN(k * state_len, state_len));
        const Eigen::VectorXd uc
            = traj_control_mid_vars(Eigen::seqN(k * control_len, control_len));
        const Eigen::VectorXd fc = cartpoleDyn(xc, uc, tc, model);

        traj.push_back({.time = tc,
                        .q = xc(Eigen::seqN(0, state_len / 2)),
                        .dq = xc(Eigen::seqN(state_len / 2, state_len / 2)),
                        .ddq = fc(Eigen::seqN(state_len / 2, state_len / 2))});
    }

    const int N = num_segments;
    const double tN = start_time + N * dt_segment;
    const Eigen::VectorXd xN
        = traj_state_vars(Eigen::seqN(N * state_len, state_len));
    const Eigen::VectorXd uN
        = traj_control_vars(Eigen::seqN(N * control_len, control_len));
    const Eigen::VectorXd fN = cartpoleDyn(xN, uN, tN, model);

    traj.push_back({.time = tN,
                    .q = xN(Eigen::seqN(0, state_len / 2)),
                    .dq = xN(Eigen::seqN(state_len / 2, state_len / 2)),
                    .ddq = fN(Eigen::seqN(state_len / 2, state_len / 2))});

    return traj;
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
    // const Eigen::VectorXd state_start{{d, std::numbers::pi, 0.0, 0.0}};
    const Eigen::VectorXd state_start = Eigen::VectorXd::Zero(state_len);
    ifopt::Component::VecBound state_bounds = createStateBounds(num_state_vars,
                                                                state_len,
                                                                d_max,
                                                                state_start,
                                                                state_end);

    // initialize knot points and midpoints
    Eigen::VectorXd state_col_init
        = Eigen::VectorXd::Zero((num_segments + 1) * state_len);
    Eigen::VectorXd state_mid_init
        = Eigen::VectorXd::Zero(num_segments * state_len);

    guessStateTraj(state_len,
                   num_segments,
                   state_start,
                   state_end,
                   state_col_init,
                   state_mid_init);

    auto traj_state_vars
        = std::make_shared<TrajectoryVariables>("traj_state_vars",
                                                std::move(state_col_init),
                                                std::move(state_bounds));
    nlp.AddVariableSet(traj_state_vars);

    // control bounds
    const int control_len = 1;
    const int num_control_vars = control_len * (num_segments + 1);
    const double max_control_force = 100;
    ifopt::Component::VecBound control_bounds
        = createControlBounds(num_control_vars, max_control_force);

    // init guess for control variables
    auto control_init = Eigen::VectorXd::Zero(num_control_vars);
    auto traj_control_vars
        = std::make_shared<TrajectoryVariables>("traj_control_vars",
                                                std::move(control_init),
                                                std::move(control_bounds));
    nlp.AddVariableSet(traj_control_vars);

    // midpoint state variables
    // one per segment
    const int num_state_mid_vars = num_segments * state_len;
    auto state_mid_bounds
        = createMidpointStateBounds(num_state_mid_vars, state_len, d_max);

    auto traj_state_mid_vars
        = std::make_shared<TrajectoryVariables>("traj_state_mid_vars",
                                                std::move(state_mid_init),
                                                std::move(state_mid_bounds));
    nlp.AddVariableSet(traj_state_mid_vars);

    // midpoint control variables
    // one per segment
    const int num_control_mid_vars = num_segments * control_len;
    auto control_mid_bounds
        = createControlBounds(num_control_mid_vars, max_control_force);
    auto control_mid_init = Eigen::VectorXd::Zero(num_control_mid_vars);
    auto traj_control_mid_vars
        = std::make_shared<TrajectoryVariables>("traj_control_mid_vars",
                                                std::move(control_mid_init),
                                                std::move(control_mid_bounds));
    nlp.AddVariableSet(traj_control_mid_vars);

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
    const int num_hermite_constraints = state_len * num_segments;
    const int num_simpson_constraints = state_len * num_segments;

    const auto hermite_constraints
        = std::make_shared<HermiteMidpointConstraints>(num_hermite_constraints,
                                                       traj_state_vars,
                                                       state_len,
                                                       traj_control_vars,
                                                       traj_state_mid_vars,
                                                       traj_control_mid_vars,
                                                       control_len,
                                                       dt_segment,
                                                       dyn_fn,
                                                       jac_dyn_wrt_state_fn,
                                                       jac_dyn_wrt_control_fn);

    const auto simpson_constraints
        = std::make_shared<SimpsonDefectConstraints>(num_simpson_constraints,
                                                     traj_state_vars,
                                                     state_len,
                                                     traj_control_vars,
                                                     traj_state_mid_vars,
                                                     traj_control_mid_vars,
                                                     control_len,
                                                     dt_segment,
                                                     dyn_fn,
                                                     jac_dyn_wrt_state_fn,
                                                     jac_dyn_wrt_control_fn);

    nlp.AddConstraintSet(hermite_constraints);
    nlp.AddConstraintSet(simpson_constraints);
    nlp.AddCostSet(std::make_shared<ControlEffortHermSimpCost>(
        "effort_cost",
        traj_control_vars->GetName(),
        traj_control_mid_vars->GetName(),
        control_len,
        dt_segment));

    nlp.PrintCurrent();
    std::cout << "state variables: " << std::endl;
    std::cout << traj_state_vars->GetValues().transpose() << std::endl;
    std::cout << "control variables: " << std::endl;
    std::cout << traj_control_vars->GetValues().transpose() << std::endl;
    std::cout << "Hermite midpoint constraint values:" << std::endl;
    std::cout << hermite_constraints->GetValues().transpose() << std::endl;

    std::cout << "Simpson defect constraint values:" << std::endl;
    std::cout << simpson_constraints->GetValues().transpose() << std::endl;

    // choose solver and options
    ifopt::IpoptSolver ipopt;
    ipopt.SetOption("tol", 1e-3);
    ipopt.SetOption("max_iter", 3000);
    ipopt.SetOption("max_cpu_time", 60.0);
    ipopt.SetOption("print_level", 5);
    ipopt.SetOption("print_frequency_time", 3.0);
    ipopt.SetOption("derivative_test", "first-order");
    ipopt.SetOption("mu_strategy", "adaptive");
    ipopt.SetOption("output_file", "ipopt.out");

    // solve
    ipopt.Solve(nlp);
    nlp.PrintCurrent();

    std::cout << "state variables: " << std::endl;
    std::cout << traj_state_vars->GetValues().transpose() << std::endl;
    std::cout << "control variables: " << std::endl;
    std::cout << traj_control_vars->GetValues().transpose() << std::endl;
    std::cout << "Hermite midpoint constraint values:" << std::endl;
    std::cout << hermite_constraints->GetValues().transpose() << std::endl;

    std::cout << "Simpson defect constraint values:" << std::endl;
    std::cout << simpson_constraints->GetValues().transpose() << std::endl;


    const double start_time = 0.0;
    const double sample_period = 0.020;

    const Eigen::VectorXd solved_state_vars = traj_state_vars->GetValues();
    const Eigen::VectorXd solved_state_mid_vars
        = traj_state_mid_vars->GetValues();
    const Eigen::VectorXd solved_control_vars = traj_control_vars->GetValues();
    const Eigen::VectorXd solved_control_mid_vars
        = traj_control_mid_vars->GetValues();

    ///////////////////////////////////////////////////////////////////////
    // Extract/create trajectories and save to files
    //////////////////////////////////////////////////////////////////////
    HermSimpTrajExtractor traj_extractor(start_time,
                                            traj_dur,
                                            traj_state_vars->GetValues(),
                                            traj_state_mid_vars->GetValues(),
                                            state_len,
                                            traj_control_vars->GetValues(),
                                            traj_control_mid_vars->GetValues(),
                                            control_len,
                                            dt_segment,
                                            model,
                                            cartpoleDyn);
    saveDiscreteJointStateTrajCsv(
        "collocation-state-traj-hermite-simpson-cartpole.csv",
        traj_extractor.createCollocationStateTraj(model));
    saveDiscreteJointDataTrajCsv(
        "collocation-ctrl-traj-hermite-simpson-cartpole.csv",
        traj_extractor.createCollocationCtrlTraj(model));

    // save sample trajectory to file
    //const double sample_period = 0.020;
    saveDiscreteJointStateTrajCsv(
        "sample-state-traj-hermite-simpson-cartpole.csv",
        traj_extractor.createSampledStateTraj(sample_period));
    saveDiscreteJointDataTrajCsv(
        "sample-ctrl-traj-hermite-simpson-cartpole.csv",
        traj_extractor.createSampledCtrlTraj(sample_period));

    return 0;
}
