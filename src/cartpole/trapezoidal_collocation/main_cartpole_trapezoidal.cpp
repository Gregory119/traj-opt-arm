#include <iostream>
#include <numbers>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>
#include <rapidcsv.h>

#include "control_effort_trapezoidal_cost.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "trajectory_variables.hpp"
#include "trapezoidal_collocation_constraints.hpp"
#include "trapezoidal_traj_extractor.hpp"

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
    // std::cout << "ddq_dtau = \n" << ddq_dtau << std::endl;
    for (int i{}; i < state_len / 2; ++i) {
        triplets.push_back({i + state_len / 2, 0, ddq_dtau(i, 0)});
    }
    ifopt::Component::Jacobian jac(state_len, 1);
    jac.setFromTriplets(triplets.cbegin(), triplets.cend());
    return jac;
}

Eigen::VectorXd guessStateTraj(const int state_len,
                               const int num_segments,
                               const Eigen::VectorXd &state_start,
                               const Eigen::VectorXd &state_end)
{
    const int num_time_pts = num_segments + 1;
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(num_time_pts * state_len);
    // linearly interpolate from start state to end state
    for (int k{}; k < num_time_pts; ++k) {
        auto statek = ret(Eigen::seqN(k * state_len, state_len));
        const double alpha
            = static_cast<double>(k)
              / (num_time_pts - 1);  // trajectory progress factor
        statek = alpha * (state_end - state_start) + state_start;
    }
    return ret;
}

// save sample trajectory to file
void saveDiscreteJointStateTrajCsv(const std::string &filename,
                                   const DiscreteJointStateTraj &sample_traj)
{
    // time | q(0) | ... | q(n-1) | dq(0) | ... | dq(n-1) | ddq(0) | ... |
    // ddq(n-1)
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

void saveDiscreteJointDataTrajCsv(const std::string &filename,
                                  const DiscreteJointDataTraj &sample_traj)
{
    // time | data(0) | data(1) | ... | data(n-1)
    rapidcsv::Document doc{};

    // create header
    doc.InsertColumn(0, std::vector<double>(), "time");
    for (int i{}; i < sample_traj[0].data.size(); ++i) {
        std::ostringstream os;
        os << "data" << i;
        doc.InsertColumn(i + 1, std::vector<double>(), os.str());
    }

    // fill in data in rows
    for (int i{}; i < sample_traj.size(); ++i) {
        std::vector<double> row_data;
        const JointData &e = sample_traj.at(i);
        row_data.push_back(e.time);
        row_data.insert(row_data.cend(), e.data.cbegin(), e.data.cend());
        doc.InsertRow(i, row_data);
    }
    doc.Save(filename);
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
    const double start_time = 0.0;
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

    // init guess for state variables
    auto state_init
        = guessStateTraj(state_len, num_segments, state_start, state_end);
    auto traj_state_vars
        = std::make_shared<TrajectoryVariables>("traj_state_vars",
                                                std::move(state_init),
                                                std::move(state_bounds));
    nlp.AddVariableSet(traj_state_vars);

    // control bounds
    const int control_len = 1;
    const int num_control_vars = control_len * (num_segments + 1);
    const double max_control_force = 100.0;
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
    const auto col_constraints
        = std::make_shared<TrapezoidalCollocationConstraints>(
            num_constraints,
            traj_state_vars,
            state_len,
            traj_control_vars,
            control_len,
            dt_segment,
            dyn_fn,
            jac_dyn_wrt_state_fn,
            jac_dyn_wrt_control_fn);
    nlp.AddConstraintSet(col_constraints);
    nlp.AddCostSet(std::make_shared<ControlEffortTrapezoidalCost>(
        "effort_cost",
        traj_control_vars->GetName(),
        control_len,
        dt_segment));

    nlp.PrintCurrent();
    std::cout << "state variables: " << std::endl;
    std::cout << traj_state_vars->GetValues().transpose() << std::endl;
    std::cout << "control variables: " << std::endl;
    std::cout << traj_control_vars->GetValues().transpose() << std::endl;
    std::cout << "collocation constraint values:" << std::endl;
    std::cout << col_constraints->GetValues().transpose() << std::endl;

    // choose solver and options
    ifopt::IpoptSolver ipopt;
    ipopt.SetOption("tol", 1e-3);
    ipopt.SetOption("max_iter", 3000);
    ipopt.SetOption("max_cpu_time", 60.0);
    // ipopt.SetOption("print_level", 5);
    // ipopt.SetOption("print_frequency_time", 3.0);
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
    std::cout << "collocation constraint values:" << std::endl;
    std::cout << col_constraints->GetValues().transpose() << std::endl;

    ///////////////////////////////////////////////////////////////////////
    // Extract/create trajectories and save to files
    //////////////////////////////////////////////////////////////////////
    TrapezoidalTrajExtractor traj_extractor(start_time,
                                            traj_dur,
                                            traj_state_vars->GetValues(),
                                            state_len,
                                            traj_control_vars->GetValues(),
                                            control_len,
                                            dt_segment,
                                            model,
                                            cartpoleDyn);
    saveDiscreteJointStateTrajCsv(
        "collocation-state-traj-trapezoidal-cartpole.csv",
        traj_extractor.createCollocationStateTraj(model));
    saveDiscreteJointDataTrajCsv(
        "collocation-ctrl-traj-trapezoidal-cartpole.csv",
        traj_extractor.createCollocationCtrlTraj(model));

    // save sample trajectory to file
    const double sample_period = 0.020;
    saveDiscreteJointStateTrajCsv(
        "sample-state-traj-trapezoidal-cartpole.csv",
        traj_extractor.createSampledStateTraj(sample_period));
    saveDiscreteJointDataTrajCsv(
        "sample-ctrl-traj-trapezoidal-cartpole.csv",
        traj_extractor.createSampledCtrlTraj(sample_period));

    return 0;
}
