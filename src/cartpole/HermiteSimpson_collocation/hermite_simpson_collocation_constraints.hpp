#pragma once

#include <ifopt/constraint_set.h>

#include "trajectory_variables.hpp"

// A utility function to convert a sparse matrix to a vector of
// triplets. row_start and col_start are used to offset the row and column
// indices in order to use them for constructing a larger sparse matrix that
// contains the provided one as a submatrix.
std::vector<Eigen::Triplet<double>> sparseMatrixToTriplets(
    const ifopt::Component::Jacobian &mat,
    const int row_start,
    const int col_start);

// todo: consider the final time as an optimization variable in order to support
// minimizing total time of a trajectory

using DynFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &state,
                                            const Eigen::VectorXd &control,
                                            const double time)>;

using JacobianDynFn
    = std::function<ifopt::Component::Jacobian(const Eigen::VectorXd &state,
                                               const Eigen::VectorXd &control,
                                               const double time)>;

class HermiteMidpointConstraints final : public ifopt::ConstraintSet
{
public:
    /*
     * Hermite midpoint constraints for Kelly Eq. (4.3):
     *   x_c,k - 0.5 (x_k + x_{k+1}) - (h/8) (f_k - f_{k+1}) = 0
     *
     * Reference: Kelly, "An Introduction to Trajectory Optimization:
     * How to Do Your Own Direct Collocation".
     *
     * @param num_constraints Total number of scalar Hermite equations.
     *   This should be state_len * num_segments.
     */
    HermiteMidpointConstraints(
        const int num_constraints,
        const std::shared_ptr<TrajectoryVariables> &state_vars,
        const int state_len,
        const std::shared_ptr<TrajectoryVariables> &ctrl_vars,
        const std::shared_ptr<TrajectoryVariables> &state_mid_vars,
        const std::shared_ptr<TrajectoryVariables> &ctrl_mid_vars,
        const int control_len,
        const double dt_segment,
        const DynFn &dyn_fn,
        const JacobianDynFn &jac_dyn_wrt_state_fn,
        const JacobianDynFn &jac_dyn_wrt_control_fn);

    Eigen::VectorXd GetValues() const override;
    ifopt::Component::VecBound GetBounds() const override
    {
        return ifopt::Component::VecBound(GetRows(), {0.0, 0.0});
    }

    void FillJacobianBlock(
        std::string var_set,
        ifopt::Component::Jacobian &jac_block) const override;

private:
    enum class VariableType
    {
        STATE,
        CONTROL,
        STATE_MID,
        CONTROL_MID
    };

    // Create the jacobian of the constraints w.r.t the specified variable
    // types.
    void FillJacobianWrt(const VariableType var_type,
                         ifopt::Component::Jacobian &jac_block) const;

    int getVarTypeLen(const VariableType var_type) const;

    // Create the jacobian of defect constraint vector k w.r.t the vector
    // variable type (eg. state, control defects and state,control midpoints) at
    // segment k ,knot point j
    ifopt::Component::Jacobian jacConstraintsWrtVar(
        const VariableType var_type,
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const;

    // Create the jacobian of defect constraint vector k w.r.t the state vector
    // at time point j.
    ifopt::Component::Jacobian jacConstraintsWrtState(
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const;

    // Create the jacobian of defect constraint vector k w.r.t the control
    // vector at time point j.
    ifopt::Component::Jacobian jacConstraintsWrtControl(
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const;

    const std::shared_ptr<TrajectoryVariables> m_state_vars;
    const int m_state_len;
    const std::shared_ptr<TrajectoryVariables> m_ctrl_vars;
    const int m_control_len;
    const std::shared_ptr<TrajectoryVariables> m_state_mid_vars;
    const std::shared_ptr<TrajectoryVariables> m_ctrl_mid_vars;
    const double m_dt_segment;
    const DynFn m_dyn_fn;
    const JacobianDynFn m_jac_dyn_wrt_state_fn;
    const JacobianDynFn m_jac_dyn_wrt_control_fn;
    int m_num_segments;
};

class SimpsonDefectConstraints final : public ifopt::ConstraintSet
{
public:
    /*
     * Simpson defect constraints for Kelly Eq. (4.4):
     *   x_{k+1} - x_k - (h/6) (f_k + 4 f_c,k + f_{k+1}) = 0
     *
     * Reference: Kelly, "An Introduction to Trajectory Optimization:
     * How to Do Your Own Direct Collocation".
     *
     * @param num_constraints Total number of scalar Simpson equations.
     *   This should be state_len * num_segments.
     */
    SimpsonDefectConstraints(
        const int num_constraints,
        const std::shared_ptr<TrajectoryVariables> &state_vars,
        const int state_len,
        const std::shared_ptr<TrajectoryVariables> &ctrl_vars,
        const std::shared_ptr<TrajectoryVariables> &state_mid_vars,
        const std::shared_ptr<TrajectoryVariables> &ctrl_mid_vars,
        const int control_len,
        const double dt_segment,
        const DynFn &dyn_fn,
        const JacobianDynFn &jac_dyn_wrt_state_fn,
        const JacobianDynFn &jac_dyn_wrt_control_fn);

    Eigen::VectorXd GetValues() const override;
    ifopt::Component::VecBound GetBounds() const override
    {
        return ifopt::Component::VecBound(GetRows(), {0.0, 0.0});
    }

    void FillJacobianBlock(
        std::string var_set,
        ifopt::Component::Jacobian &jac_block) const override;

private:
    enum class VariableType
    {
        STATE,
        CONTROL,
        STATE_MID,
        CONTROL_MID
    };

    // Create the jacobian of the constraints w.r.t the specified variable
    // types.
    void FillJacobianWrt(const VariableType var_type,
                         ifopt::Component::Jacobian &jac_block) const;

    int getVarTypeLen(const VariableType var_type) const;

    // Create the jacobian of defect constraint vector k w.r.t the vector
    // variable type (eg. state, control defects and state,control midpoints) at
    // segment k ,knot point j
    ifopt::Component::Jacobian jacConstraintsWrtVar(
        const VariableType var_type,
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const;

    // Create the jacobian of defect constraint vector k w.r.t the state vector
    // at time point j.
    ifopt::Component::Jacobian jacConstraintsWrtState(
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const;

    // Create the jacobian of defect constraint vector k w.r.t the control
    // vector at time point j.
    ifopt::Component::Jacobian jacConstraintsWrtControl(
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const;

    const std::shared_ptr<TrajectoryVariables> m_state_vars;
    const int m_state_len;
    const std::shared_ptr<TrajectoryVariables> m_ctrl_vars;
    const int m_control_len;
    const std::shared_ptr<TrajectoryVariables> m_state_mid_vars;
    const std::shared_ptr<TrajectoryVariables> m_ctrl_mid_vars;
    const double m_dt_segment;
    const DynFn m_dyn_fn;
    const JacobianDynFn m_jac_dyn_wrt_state_fn;
    const JacobianDynFn m_jac_dyn_wrt_control_fn;
    int m_num_segments;
};