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

class TrapezoidalCollocationConstraints final : public ifopt::ConstraintSet
{
public:
    // calback signature for evaluating the dynamics
    using DynFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &state,
                                                const Eigen::VectorXd &control,
                                                const double time)>;
    // callback signature for evaluating the jacobian of the dynamics w.r.t one
    // of its inputs (eg. state, control, or time)
    using JacobianDynFn = std::function<ifopt::Component::Jacobian(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time)>;

    /*
     * @param num_constraints This is the total number of constraint equations
     *   produced by all of the defect constraint equations. Each defect
     *   constraint equation uses vectors, representing a state length number of
     *   equations. Therefore, the total number of defect constraints equals (#
     *   number of defect constraint equations) * (state length).
     * @param state_len The number of elements in a state vector at a particular
     *   time.
     * @param control_len The number of elements in a control vector at a
     *   particular time.
     * @param dt_segment The fixed duration of every time segement.
     * @param dyn_fn Callback function to get the value of the dynamics
     *   function.
     * @param jac_dyn_wrt_state_fn Callback function to get the value of the
     *   jacobian of the dynamics function w.r.t the input state.
     * @param jac_dyn_wrt_control_fn Callback function to get the value of the
     *   jacobian of the dynamics function w.r.t the input state.
     */
    TrapezoidalCollocationConstraints(
        const int num_constraints,
        const std::shared_ptr<TrajectoryVariables> &state_vars,
        const int state_len,
        const std::shared_ptr<TrajectoryVariables> &ctrl_vars,
        const int control_len,
        const double dt_segment,
        const DynFn &dyn_fn,
        const JacobianDynFn &jac_dyn_wrt_state_fn,
        const JacobianDynFn &jac_dyn_wrt_control_fn);
    // Get the current values of all constraints
    Eigen::VectorXd GetValues() const override;

    ifopt::Component::VecBound GetBounds() const override
    {
        // defects should all be zero
        ifopt::Component::VecBound bounds(GetRows(), {0.0, 0.0});
        return bounds;
    }

    // Create the jacobian of the contraints w.r.t all of the optimization
    // variables (state, control).
    void FillJacobianBlock(
        std::string var_set,
        ifopt::Component::Jacobian &jac_block) const override;

private:
    enum class VariableType
    {
        STATE,
        CONTROL
    };

    // Create the jacobian of the constraints w.r.t the specified variable
    // types.
    void FillJacobianWrt(const VariableType var_type,
                         ifopt::Component::Jacobian &jac_block) const;

    int getVarTypeLen(const VariableType var_type) const;

    // Create the jacobian of defect constraint vector k w.r.t the vector
    // variable type (eg. state or control) at time point j.
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
    const double m_dt_segment;
    const DynFn m_dyn_fn;
    const JacobianDynFn m_jac_dyn_wrt_state_fn;
    const JacobianDynFn m_jac_dyn_wrt_control_fn;
    int m_num_segments;
};
