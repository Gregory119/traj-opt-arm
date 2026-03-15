#include "trapezoidal_traj_extractor.hpp"

#include <linear_spline.hpp>
#include <quadratic_spline.hpp>

namespace pin = pinocchio;

TrapezoidalTrajExtractor::TrapezoidalTrajExtractor(
    const double start_time,
    const double traj_dur,
    const Eigen::VectorXd &state_vars,
    const int state_len,
    const Eigen::VectorXd &ctrl_vars,
    const int ctrl_len,
    const double dt_segment,
    const pin::Model &model,
    const DynFn &dyn_fn)
    : m_start_time{start_time}
    , m_dur{traj_dur}
    , m_state_vars{state_vars}
    , m_state_len{state_len}
    , m_ctrl_vars{ctrl_vars}
    , m_ctrl_len{ctrl_len}
    , m_dt_segment{dt_segment}
    , m_dyn_fn{dyn_fn}
    , m_dyn_vals{createDynVals(model)}
{}

DiscreteJointStateTraj TrapezoidalTrajExtractor::createCollocationStateTraj(
    const pin::Model &model)
{
    DiscreteJointStateTraj traj;
    const int num_samples = m_state_vars.size() / m_state_len;
    for (int i{}; i < num_samples; ++i) {
        const double time = i * m_dt_segment + m_start_time;
        const Eigen::VectorXd state
            = m_state_vars(Eigen::seqN(i * m_state_len, m_state_len));
        const Eigen::VectorXd dstate_dt
            = m_dyn_vals(Eigen::seqN(i * m_state_len, m_state_len));
        traj.push_back(
            {.time = time,
             .q = state(Eigen::seqN(0, m_state_len / 2)),
             .dq = state(Eigen::seqN(m_state_len / 2, m_state_len / 2)),
             .ddq = dstate_dt(Eigen::seqN(m_state_len / 2, m_state_len / 2))});
    }
    return traj;
}

DiscreteJointDataTraj TrapezoidalTrajExtractor::createCollocationCtrlTraj(
    const pinocchio::Model &model)
{
    DiscreteJointDataTraj traj;
    const int num_samples = m_ctrl_vars.size() / m_ctrl_len;
    for (int i{}; i < num_samples; ++i) {
        const double time = i * m_dt_segment + m_start_time;
        const Eigen::VectorXd ctrl
            = m_ctrl_vars(Eigen::seqN(i * m_ctrl_len, m_ctrl_len));
        traj.push_back({.time = time, .data = ctrl});
    }
    return traj;
}

DiscreteJointStateTraj TrapezoidalTrajExtractor::createSampledStateTraj(
    const double sample_period)
{
    // create quadratic spline
    const QuadraticSpline state_spline = createStateSpline();

    const LinearSpline dyn_spline = createDynSpline();

    // create sample trajectory
    return createDiscreteJointStateTraj(sample_period,
                                        state_spline,
                                        dyn_spline);
}

DiscreteJointDataTraj TrapezoidalTrajExtractor::createSampledCtrlTraj(
    const double sample_period)
{
    // create control spline
    std::vector<Eigen::VectorXd> ctrl_vals;
    for (size_t i{}; i < m_ctrl_len; ++i) {
        const Eigen::VectorXd ctrl
            = m_ctrl_vars(Eigen::seqN(i * m_ctrl_len, m_ctrl_len));
        ctrl_vals.push_back(ctrl);
    }
    const LinearSpline ctrl_spline(ctrl_vals, m_start_time, m_dur);

    // sample spline
    DiscreteJointDataTraj sampled_traj;
    const int num_samples = static_cast<int>(m_dur / sample_period) + 1;
    for (int i{}; i < num_samples; ++i) {
        const double time = i * sample_period + m_start_time;
        sampled_traj.push_back(
            JointData{.time = time, .data = ctrl_spline.getValue(time)});
    }
    return sampled_traj;
}

QuadraticSpline TrapezoidalTrajExtractor::createStateSpline()
{
    std::vector<Eigen::VectorXd> state_vals;
    std::vector<Eigen::VectorXd> state_grad_vals;
    const int num_state_vecs = m_state_vars.size() / m_state_len;
    for (int i{}; i < num_state_vecs; ++i) {
        const Eigen::VectorXd state
            = m_state_vars(Eigen::seqN(i * m_state_len, m_state_len));
        const Eigen::VectorXd dstate_dt
            = m_dyn_vals(Eigen::seqN(i * m_state_len, m_state_len));
        state_vals.push_back(state);
        state_grad_vals.push_back(dstate_dt);
    }

    return QuadraticSpline(state_vals, state_grad_vals, m_start_time, m_dur);
}

LinearSpline TrapezoidalTrajExtractor::createDynSpline()
{
    std::vector<Eigen::VectorXd> state_grad_vals;
    const int num_state_vecs = m_dyn_vals.size() / m_state_len;
    for (int i{}; i < num_state_vecs; ++i) {
        const Eigen::VectorXd dstate_dt
            = m_dyn_vals(Eigen::seqN(i * m_state_len, m_state_len));
        state_grad_vals.push_back(dstate_dt);
    }
    return LinearSpline(state_grad_vals, m_start_time, m_dur);
}

DiscreteJointStateTraj TrapezoidalTrajExtractor::createDiscreteJointStateTraj(
    const double sample_period,
    const QuadraticSpline &state_spline,
    const LinearSpline &dyn_spline)
{
    DiscreteJointStateTraj sample_traj;
    const int num_samples = static_cast<int>(m_dur / sample_period) + 1;
    for (int i{}; i < num_samples; ++i) {
        const double time = i * sample_period + m_start_time;
        const Eigen::VectorXd state = state_spline.getValue(time);
        const Eigen::VectorXd dstate_dt = dyn_spline.getValue(time);
        sample_traj.push_back(
            {.time = time,
             .q = state(Eigen::seqN(0, state.size() / 2)),
             .dq = state(Eigen::seqN(state.size() / 2, state.size() / 2)),
             .ddq
             = dstate_dt(Eigen::seqN(state.size() / 2, state.size() / 2))});
    }
    return sample_traj;
}

Eigen::VectorXd TrapezoidalTrajExtractor::createDynVals(const pin::Model &model)
{
    Eigen::VectorXd dyn_vals = Eigen::VectorXd::Zero(m_state_vars.size());
    const int num_state_vecs = m_state_vars.size() / m_state_len;
    for (int i{}; i < num_state_vecs; ++i) {
        const Eigen::VectorXd state
            = m_state_vars(Eigen::seqN(i * m_state_len, m_state_len));
        const Eigen::VectorXd ctrl
            = m_ctrl_vars(Eigen::seqN(i * m_ctrl_len, m_ctrl_len));
        const double time = i * m_dt_segment;
        dyn_vals(Eigen::seqN(i * m_state_len, m_state_len))
            = m_dyn_fn(state, ctrl, time, model);
    }
    return dyn_vals;
}
