#include <iostream>
#include <numbers>
#include <pinocchio/parsers/mjcf.hpp>
#include <so101_bus.hpp>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include "control_effort_trapezoidal_cost.hpp"
#include "robot_dynamics.hpp"
#include "save_trajectory.hpp"
#include "simulator.hpp"
#include "trajectory_variables.hpp"
#include "trapezoidal_collocation_constraints.hpp"
#include "trapezoidal_traj_extractor.hpp"

namespace pin = pinocchio;

/*
 * Create an upper and lower bound for each state vector along the trajectory.
 */
ifopt::Component::VecBound createStateBounds(const int num_state_vars,
                                             const int state_len,
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

            // joint positions path bounds
            for (int j{}; j < state_len / 2 - 1; ++j) {
                bounds.push_back({-1.0 / 4.0 * std::numbers::pi,
                                  1.0 / 4.0 * std::numbers::pi});
            }
            // end effector path bounds
            bounds.push_back({0.0, 2.25});
            
            // joint velocity path bounds
            for (int j{}; j < state_len / 2; ++j) {
                bounds.push_back({-ifopt::inf, ifopt::inf});
            }
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

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cout << "Path to model required." << std::endl;
        return 0;
    }

    // Load the urdf model
    const std::string mj_filename = argv[1];
    pin::Model model;
    pin::mjcf::buildModel(mj_filename, model);
    std::cout << "model name: " << model.name << std::endl;

    // define problem
    ifopt::Problem nlp;
    const double start_time = 0.0;
    const double traj_dur = 2.0;
    const int num_segments = 10;
    const double dt_segment = traj_dur / num_segments;

    // state bounds
    const int state_len = 6 * 2;
    const int num_state_vars = (num_segments + 1) * state_len;
    const Eigen::VectorXd state_end{{-std::numbers::pi / 4,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0}};
    const Eigen::VectorXd state_start = Eigen::VectorXd::Zero(state_len);
    ifopt::Component::VecBound state_bounds
        = createStateBounds(num_state_vars, state_len, state_start, state_end);

    // init guess for state variables
    auto state_init
        = guessStateTraj(state_len, num_segments, state_start, state_end);
    auto traj_state_vars
        = std::make_shared<TrajectoryVariables>("traj_state_vars",
                                                std::move(state_init),
                                                state_bounds);
    nlp.AddVariableSet(traj_state_vars);

    // control bounds
    const int control_len = 6;
    const int num_control_vars = control_len * (num_segments + 1);
    const double rated_torque_kgcm = 10;
    const double gravity = 9.81;
    const double max_control_force = rated_torque_kgcm * gravity / 100.0;
    ifopt::Component::VecBound control_bounds
        = createControlBounds(num_control_vars, max_control_force);

    // init guess for control variables
    auto control_init = Eigen::VectorXd::Zero(num_control_vars);
    auto traj_control_vars
        = std::make_shared<TrajectoryVariables>("traj_control_vars",
                                                std::move(control_init),
                                                control_bounds);
    nlp.AddVariableSet(traj_control_vars);

    // add constraints
    const auto dyn_fn
        = [&](const Eigen::VectorXd &state,
              const Eigen::VectorXd &control,
              const double time) { return dyn(state, control, time, model); };
    const auto jac_dyn_wrt_state_fn = [&](const Eigen::VectorXd &state,
                                          const Eigen::VectorXd &control,
                                          const double time) {
        return jacDynWrtState(state, control, time, model);
    };
    const auto jac_dyn_wrt_control_fn = [&](const Eigen::VectorXd &state,
                                            const Eigen::VectorXd &control,
                                            const double time) {
        return jacDynWrtControl(state, control, time, model);
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
                                            dyn);
    saveDiscreteJointStateTrajCsv(
        "collocation-state-traj-trapezoidal-so101.csv",
        traj_extractor.createCollocationStateTraj(model));
    saveDiscreteJointDataTrajCsv(
        "collocation-ctrl-traj-trapezoidal-so101.csv",
        traj_extractor.createCollocationCtrlTraj(model));

    // save sample trajectory to file
    const double sample_period = 0.020;
    const DiscreteJointStateTraj sampled_state_traj
        = traj_extractor.createSampledStateTraj(sample_period);
    saveDiscreteJointStateTrajCsv("sample-state-traj-trapezoidal-so101.csv",
                                  sampled_state_traj);
    saveDiscreteJointDataTrajCsv(
        "sample-ctrl-traj-trapezoidal-so101.csv",
        traj_extractor.createSampledCtrlTraj(sample_period));

    // save bounds
    saveColBoundsCsv("state-traj-bounds-trapezoidal-so101.csv",
                     state_bounds,
                     state_len,
                     start_time,
                     traj_dur);
    saveColBoundsCsv("ctrl-traj-bounds-trapezoidal-so101.csv",
                     control_bounds,
                     control_len,
                     start_time,
                     traj_dur);

    ///////////////////////////////////////////////////////////////////////
    // Send trajectory to simulated robot
    //////////////////////////////////////////////////////////////////////
    Simulator::getInstance()->setTrajectory(sampled_state_traj);

    Simulator::getInstance()->run();

    ///////////////////////////////////////////////////////////////////////
    // Send trajectory to physical arm
    //////////////////////////////////////////////////////////////////////
    if (false) {
        return 0;
    }

    SO101Bus::Config cfg;
    cfg.device = "/dev/ttyACM0";
    cfg.record_timing_stats = true;
    cfg.sid_to_pos_tic_range = {{
      {1, ServoPosRange{751, 3470}},
      {2, ServoPosRange{920, 3281}},
      {3, ServoPosRange{933, 3137}},
      {4, ServoPosRange{875, 3215}},
      {5, ServoPosRange{221, 4022}},
      {6, ServoPosRange{2037, 3499}},
    }};

    SO101Bus bus(cfg);
    if (!bus.connect()) {
        std::cerr << "failed to connect to " << cfg.device << "\n";
        return 0;
    }

    if (!bus.execute_traj_full(sampled_state_traj, PosUnit::RADIAN)) {
        std::cerr << "trajectory execution failed\n";
        return 2;
    }

    return 0;
}
