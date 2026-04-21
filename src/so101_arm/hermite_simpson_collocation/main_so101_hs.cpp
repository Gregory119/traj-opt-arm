#include <iostream>
#include <numbers>
#include <pinocchio/parsers/mjcf.hpp>
#include <so101_bus.hpp>

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include "control_effort_hs_cost.hpp"
#include "hermite_simpson_collocation_constraints.hpp"
#include "hs_traj_extractor.hpp"
#include "robot_dynamics.hpp"
#include "save_trajectory.hpp"
#include "trajectory_variables.hpp"
#include "save_trajectory.hpp"
#include "simulator.hpp"

namespace pin = pinocchio;

/*
 * Create an upper and lower bound for each state vector along the trajectory.
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

ifopt::Component::VecBound createMidpointStateBounds(const int num_state_vars,
                                                     const int state_len,
                                                     const double d_max)
{
    ifopt::Component::VecBound bounds;
    bounds.reserve(num_state_vars);

    for (int i = 0; i < num_state_vars; i += state_len) {
        for (int j = 0; j < state_len; ++j) {
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
            auto state_mid_i
                = state_mid_init(Eigen::seqN(k * state_len, state_len));
            const double alpha_mid = ((2 * k) - 1) / (2 * num_segments);
            state_mid_i = alpha_mid * (state_start - state_end) + state_start;
        }
    }
    return;
}



int main(int argc, char **argv)
{
    if (argc != 3) {
        std::cout << "Path to model and calibration file required (in this order)." << std::endl;
        return 0;
    }
    const std::string calibration_file_path(argv[2]);

    // Load the urdf model
    const std::string mj_filename = argv[1];
    pin::Model model;
    pin::mjcf::buildModel(mj_filename, model);
    std::cout << "model name: " << model.name << std::endl;

    // define problem
    ifopt::Problem nlp;
    const double traj_dur = 2.0;
    const int num_segments = 10;
    const double dt_segment = traj_dur / num_segments;
    const double start_time = 0.0;

    // final q0 position
    const double d = 0.8;

    // state bounds
    const double d_max = 2 * d;
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
    // const Eigen::VectorXd state_start{{d, std::numbers::pi, 0.0, 0.0}};
    const Eigen::VectorXd state_start = Eigen::VectorXd::Zero(state_len);
    ifopt::Component::VecBound state_bounds = createStateBounds(num_state_vars,
                                                                state_len,
                                                                d_max,
                                                                state_start,
                                                                state_end);
    const auto state_bounds_for_save = state_bounds;

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
    const int control_len = 6;
    const int num_control_vars = control_len * (num_segments + 1);
    const double rated_torque_kgcm = 10 / 1.2;
    const double gravity = 9.81;
    const double max_control_force = rated_torque_kgcm * gravity / 100.0;
    ifopt::Component::VecBound control_bounds
        = createControlBounds(num_control_vars, max_control_force);
    const auto control_bounds_for_save = control_bounds;

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
    const auto state_mid_bounds_for_save = state_mid_bounds;

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
    const auto control_mid_bounds_for_save = control_mid_bounds;
    auto control_mid_init = Eigen::VectorXd::Zero(num_control_mid_vars);
    auto traj_control_mid_vars
        = std::make_shared<TrajectoryVariables>("traj_control_mid_vars",
                                                std::move(control_mid_init),
                                                std::move(control_mid_bounds));
    nlp.AddVariableSet(traj_control_mid_vars);

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
                                         dyn);
    saveDiscreteJointStateTrajCsv(
        "collocation-state-traj-hermite-simpson-so101.csv",
        traj_extractor.createCollocationStateTraj(model));
    saveDiscreteJointDataTrajCsv(
        "collocation-ctrl-traj-hermite-simpson-so101.csv",
        traj_extractor.createCollocationCtrlTraj(model));

    // save sample trajectory to file
    const double sample_period = 0.020;
        const DiscreteJointStateTraj sampled_state_traj
        = traj_extractor.createSampledStateTraj(sample_period);
    saveDiscreteJointStateTrajCsv(
        "sample-state-traj-hermite-simpson-so101.csv",
        sampled_state_traj);
    saveDiscreteJointDataTrajCsv(
        "sample-ctrl-traj-hermite-simpson-so101.csv",
        traj_extractor.createSampledCtrlTraj(sample_period));

    //save bounds
    saveHermiteSimpsonBoundsCsv("state-traj-bounds-hermite-simpson-so101.csv",
                                state_bounds_for_save,
                                state_mid_bounds_for_save,
                                state_len,
                                start_time,
                                traj_dur);

    saveHermiteSimpsonBoundsCsv("ctrl-traj-bounds-hermite-simpson-so101.csv",
                                control_bounds_for_save,
                                control_mid_bounds_for_save,
                                control_len,
                                start_time,
                                traj_dur);
    ///////////////////////////////////////////////////////////////////////
    // Send trajectory to simulated robot
    //////////////////////////////////////////////////////////////////////
    Simulator::getInstance()->setCsvRecordFileName("sim-record-state-traj-hermite-simpson-so101.csv");
    Simulator::getInstance()->setTrajectory(sampled_state_traj);

    Simulator::getInstance()->run();

    ///////////////////////////////////////////////////////////////////////
    // Send trajectory to physical arm
    //////////////////////////////////////////////////////////////////////
    if (false) {
        return 0;
    }

    Calibration calibration(calibration_file_path);
    SO101Bus::Config cfg(calibration);
    cfg.record_timing_stats = true;

    SO101Bus bus(cfg);
    if (!bus.connect()) {
        std::cerr << "failed to connect to " << cfg.device << "\n";
        return 0;
    }

    DiscreteJointStateTraj meas_traj;
    if (!bus.execute_traj_full(sampled_state_traj, PosUnit::RADIAN, meas_traj)) {
        std::cerr << "trajectory execution failed\n";
        return 2;
    }

    saveDiscreteJointStateTrajCsv("hw-record-state-traj-trapezoidal-so101.csv",
                                  meas_traj);

    return 0;
}