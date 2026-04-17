#include "hs_traj_extractor.hpp"

#include <quadratic_spline_hs.hpp>
#include <cubic_spline.hpp>

namespace pin = pinocchio;

HermSimpTrajExtractor::HermSimpTrajExtractor(
    const double start_time,
    const double traj_dur,
    const Eigen::VectorXd &state_vars,
    const Eigen::VectorXd &state_mid_vars,
    const int state_len,
    const Eigen::VectorXd &ctrl_vars,
    const Eigen::VectorXd &ctrl_mid_vars,
    const int ctrl_len,
    const double dt_segment,
    const pin::Model &model,
    const DynFn &dyn_fn)
    : m_start_time{start_time}
    , m_dur{traj_dur}
    , m_state_vars{state_vars}
    , m_state_mid_vars{state_mid_vars}
    , m_state_len{state_len}
    , m_ctrl_vars{ctrl_vars}
    , m_ctrl_mid_vars{ctrl_mid_vars}
    , m_ctrl_len{ctrl_len}
    , m_dt_segment{dt_segment}
    , m_dyn_fn{dyn_fn}
    , m_dyn_vals{createDynVals(model)}
{}

DiscreteJointStateTraj HermSimpTrajExtractor::createCollocationStateTraj(
    const pin::Model &model)
{
    DiscreteJointStateTraj traj;
    const int num_segments = m_state_mid_vars.size() / m_state_len;

    for (int k{}; k < num_segments; ++k) {
        const double tk = m_start_time + k * m_dt_segment;
        const double tmid = tk + 0.5 * m_dt_segment;

        const Eigen::VectorXd state_k
            = m_state_vars(Eigen::seqN(k * m_state_len, m_state_len));
        const Eigen::VectorXd dyn_k
            = m_dyn_vals(Eigen::seqN(k * m_state_len, m_state_len));
        traj.push_back(
            {.time = tk,
             .q = state_k(Eigen::seqN(0, m_state_len / 2)),
             .dq = state_k(Eigen::seqN(m_state_len / 2, m_state_len / 2)),
             .ddq = dyn_k(Eigen::seqN(m_state_len / 2, m_state_len / 2))});

        const Eigen::VectorXd state_mid
            = m_state_mid_vars(Eigen::seqN(k * m_state_len, m_state_len));
        const Eigen::VectorXd dyn_mid
            = m_dyn_mid_vals(Eigen::seqN(k * m_state_len, m_state_len));
        traj.push_back(
            {.time = tmid,
             .q = state_mid(Eigen::seqN(0, m_state_len / 2)),
             .dq = state_mid(Eigen::seqN(m_state_len / 2, m_state_len / 2)),
             .ddq = dyn_mid(Eigen::seqN(m_state_len / 2, m_state_len / 2))});
    }

    const double tf = m_start_time + num_segments * m_dt_segment;
    const Eigen::VectorXd state_f
        = m_state_vars(Eigen::seqN(num_segments * m_state_len, m_state_len));
    const Eigen::VectorXd dyn_f
        = m_dyn_vals(Eigen::seqN(num_segments * m_state_len, m_state_len));
    traj.push_back(
        {.time = tf,
         .q = state_f(Eigen::seqN(0, m_state_len / 2)),
         .dq = state_f(Eigen::seqN(m_state_len / 2, m_state_len / 2)),
         .ddq = dyn_f(Eigen::seqN(m_state_len / 2, m_state_len / 2))});

    return traj;
}

DiscreteJointDataTraj HermSimpTrajExtractor::createCollocationCtrlTraj(
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

DiscreteJointStateTraj HermSimpTrajExtractor::createSampledStateTraj(
    const double sample_period)
{
    // create quadratic spline
    const CubicSpline state_spline = createStateSpline();

    const QuadraticSpline dyn_spline = createDynSpline();

    // create sample trajectory
    return createDiscreteJointStateTraj(sample_period,
                                        state_spline,
                                        dyn_spline);
}

DiscreteJointDataTraj HermSimpTrajExtractor::createSampledCtrlTraj(
    const double sample_period)
{
    // create control spline
    std::vector<Eigen::VectorXd> ctrl_vals;
    const int num_ctrl_vecs = m_ctrl_vars.size() / m_ctrl_len;
    for (size_t i{}; i < num_ctrl_vecs; ++i) {
        const Eigen::VectorXd ctrl
            = m_ctrl_vars(Eigen::seqN(i * m_ctrl_len, m_ctrl_len));
        ctrl_vals.push_back(ctrl);
    }
    const QuadraticSpline ctrl_spline(ctrl_vals, m_start_time, m_dur);

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

CubicSpline HermSimpTrajExtractor::createStateSpline()
{
    std::vector<Eigen::VectorXd> state_vals;
    std::vector<Eigen::VectorXd> state_grad_vals;
    const int num_state_vecs = m_state_vars.size() / m_state_len;
    for (int i = 0; i < num_state_vecs; i += 2) {
        state_vals.push_back(
            m_state_vars(Eigen::seqN(i * m_state_len, m_state_len)));
        state_grad_vals.push_back(
            m_dyn_vals(Eigen::seqN(i * m_state_len, m_state_len)));
    }

    return CubicSpline(state_vals, state_grad_vals, m_start_time, m_dur);
}

QuadraticSpline HermSimpTrajExtractor::createDynSpline()
{
    std::vector<Eigen::VectorXd> dyn_vals_hs;
    const int num_segments = m_state_mid_vars.size() / m_state_len;

    for (int k{}; k < num_segments; ++k) {
        dyn_vals_hs.push_back(
            m_dyn_vals(Eigen::seqN(k * m_state_len, m_state_len)));
        dyn_vals_hs.push_back(
            m_dyn_mid_vals(Eigen::seqN(k * m_state_len, m_state_len)));
    }
    dyn_vals_hs.push_back(
        m_dyn_vals(Eigen::seqN(num_segments * m_state_len, m_state_len)));

    return QuadraticSpline(dyn_vals_hs, m_start_time, m_dur);
}

DiscreteJointStateTraj HermSimpTrajExtractor::createDiscreteJointStateTraj(
    const double sample_period,
    const CubicSpline &state_spline,
    const QuadraticSpline &dyn_spline)
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

Eigen::VectorXd HermSimpTrajExtractor::createDynVals(const pin::Model &model)
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


Eigen::VectorXd HermSimpTrajExtractor::createDynMidVals(const pin::Model &model)
{
    Eigen::VectorXd dyn_mid_vals = Eigen::VectorXd::Zero(m_state_mid_vars.size());
    const int num_state_mid_vecs = m_state_mid_vars.size() / m_state_len;

    for (int i{}; i < num_state_mid_vecs; ++i) {
        const Eigen::VectorXd state_mid
            = m_state_mid_vars(Eigen::seqN(i * m_state_len, m_state_len));
        const Eigen::VectorXd ctrl_mid
            = m_ctrl_mid_vars(Eigen::seqN(i * m_ctrl_len, m_ctrl_len));
        const double time = m_start_time
                          + (static_cast<double>(i) + 0.5) * m_dt_segment;

        dyn_mid_vals(Eigen::seqN(i * m_state_len, m_state_len))
            = m_dyn_fn(state_mid, ctrl_mid, time, model);
    }

    return dyn_mid_vals;
}