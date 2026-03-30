#pragma once

#include <Eigen/Dense>
#include <traj_element.hpp>

#include "pinocchio/multibody/model.hpp"

class QuadraticSpline;
class CubicSpline;

// Takes a Hermite-Simpson collocation solution and outputs trajectories with
// different discretization based on interpolating splines.
class HermSimpTrajExtractor
{
public:
    // calback signature for evaluating the dynamics
    using DynFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &state,
                                                const Eigen::VectorXd &control,
                                                const double time,
                                                const pinocchio::Model &model)>;

    HermSimpTrajExtractor(const double start_time,
                             const double traj_dur,
                             const Eigen::VectorXd &state_vars,
                             const Eigen::VectorXd &state_mid_vars,
                             const int state_len,
                             const Eigen::VectorXd &ctrl_vars,
                             const Eigen::VectorXd &ctrl_mid_vars,
                             const int ctrl_len,
                             const double dt_segment,
                             const pinocchio::Model &model,
                             const DynFn &dyn_fn);

    // Get collocation traj. This is equivalent to the NLP solution without any
    // post processing (no use of splines or interpolation).
    DiscreteJointStateTraj createCollocationStateTraj(
        const pinocchio::Model &model);
    DiscreteJointDataTraj createCollocationCtrlTraj(
        const pinocchio::Model &model);

    // Formed by first creating splines from the NLP solution and then
    // interpolating based on the sample period.
    DiscreteJointStateTraj createSampledStateTraj(const double sample_period);
    DiscreteJointDataTraj createSampledCtrlTraj(const double sample_period);

private:
    Eigen::VectorXd createDynVals(const pinocchio::Model &model);
    Eigen::VectorXd createDynMidVals(const pinocchio::Model &model);
    CubicSpline createStateSpline();
    QuadraticSpline createDynSpline();

    DiscreteJointStateTraj createDiscreteJointStateTraj(
        const double sample_period,
        const CubicSpline &state_spline,
        const QuadraticSpline &dstate_dt_spline);

    const double m_start_time;
    const double m_dur;
    const Eigen::VectorXd m_state_vars;
    const int m_state_len;
    const Eigen::VectorXd m_ctrl_vars;
    const int m_ctrl_len;
    const double m_dt_segment;
    const DynFn m_dyn_fn;

    const Eigen::VectorXd m_dyn_vals;
    const Eigen::VectorXd m_state_mid_vars;
    const Eigen::VectorXd m_ctrl_mid_vars;
    const Eigen::VectorXd m_dyn_mid_vals;
};