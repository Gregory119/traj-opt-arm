#pragma once

#include <ifopt/constraint_set.h>

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
    using DynFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &state,
                                                const Eigen::VectorXd &control,
                                                const double time)>;
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
     * @param segment_dt The fixed duration of every time segement.
     * @param dyn_fn Callback function to get the value of the dynamics
     *   function.
     * @param jac_dyn_fn_wrt_state Callback function to get the value of the
     * jacobian of the dynamics function w.r.t the input state.
     */
    TrapezoidalCollocationConstraints(
        const int num_constraints,
        const std::string &state_var_set_name,
        const int state_len,
        const std::string &control_var_set_name,
        const int control_len,
        const double segment_dt,
        const DynFn &dyn_fn,
        const JacobianDynFn &jac_dyn_fn_wrt_state);
    // Get the current values of all constraints
    Eigen::VectorXd GetValues() const override;

    ifopt::Component::VecBound GetBounds() const override
    {
        // defects should all be zero
        ifopt::Component::VecBound bounds(GetRows(), {0.0, 0.0});
        return bounds;
    }

    void FillJacobianBlock(std::string var_set,
                           ifopt::Component::Jacobian &jac_block) const override
    {
        if (var_set == m_state_var_set_name) {
            FillJacobianWrtState(jac_block);
        } else if (var_set == m_control_var_set_name) {
        }
    }

private:
    // Create the jacobian of the constraints w.r.t the state variables.
    void FillJacobianWrtState(ifopt::Component::Jacobian &jac_block) const;

    const std::string m_state_var_set_name;
    const int m_state_len;
    const std::string m_control_var_set_name;
    const int m_control_len;
    const double m_segment_dt;
    const DynFn m_dyn_fn;
    const JacobianDynFn m_jac_dyn_fn_wrt_state;
    int m_num_segments;
    Eigen::Vector4d m_x;
};
