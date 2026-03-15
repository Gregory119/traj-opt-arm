#pragma once

#include <Eigen/Dense>
#include <traj_element.hpp>

#include "pinocchio/multibody/model.hpp"

class QuadraticSpline;
class LinearSpline;

// Takes a trapezoidal collocation solution and outputs trajectories with
// different discretization based on interpolating splines.
class TrapezoidalTrajExtractor
{
public:
    // calback signature for evaluating the dynamics
    using DynFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &state,
                                                const Eigen::VectorXd &control,
                                                const double time,
                                                const pinocchio::Model& model)>;

    TrapezoidalTrajExtractor(const double start_time,
                             const double traj_dur,
                             const Eigen::VectorXd &state_vars,
                             const int state_len,
                             const Eigen::VectorXd &ctrl_vars,
                             const int ctrl_len,
                             const double dt_segment,
                             const pinocchio::Model &model,
                             const DynFn &dyn_fn);

    // get collocation traj
    DiscreteJointStateTraj createCollocationStateTraj(const pinocchio::Model &model);

    // DiscreteJointTorqueTraj createCollocationCtrlTraj(const pinocchio::Model
    // &model);

    // sample trajectory
    DiscreteJointStateTraj createSampledJointTraj(const double sample_period);

    // DiscreteJointTorqueTraj createSampledCtrlTraj(const double
    // sample_period);

private:
    Eigen::VectorXd createDynVals(const pinocchio::Model &model);
    QuadraticSpline createStateSpline();
    LinearSpline createDynSpline();

    DiscreteJointStateTraj createDiscreteJointStateTraj(
        const double sample_period,
        const QuadraticSpline &state_spline,
        const LinearSpline &dstate_dt_spline);

    const double m_start_time;
    const double m_dur;
    const Eigen::VectorXd m_state_vars;
    const int m_state_len;
    const Eigen::VectorXd m_ctrl_vars;
    const int m_ctrl_len;
    const double m_dt_segment;
    const DynFn m_dyn_fn;

    const Eigen::VectorXd m_dyn_vals;
};
