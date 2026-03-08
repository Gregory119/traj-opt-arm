#include "trapezoidal_collocation_constraints.hpp"

#include <iostream>

std::vector<Eigen::Triplet<double>> sparseMatrixToTriplets(
    const ifopt::Component::Jacobian &mat,
    const int row_start,
    const int col_start)
{
    std::vector<Eigen::Triplet<double>> triplets;
    // Reserve space to avoid reallocations, the number of non-zeros is a good
    // estimate
    triplets.reserve(mat.nonZeros());

    for (int i = 0; i < mat.outerSize(); ++i) {
        // Iterate over all non-zero elements in the current outer dimension
        // (column or row, depending on storage order)
        for (ifopt::Component::Jacobian::InnerIterator it(mat, i); it; ++it) {
            triplets.emplace_back(it.row() + row_start,
                                  it.col() + col_start,
                                  it.value());
        }
    }
    return triplets;
}

TrapezoidalCollocationConstraints::TrapezoidalCollocationConstraints(
    const int num_constraints,
    const std::shared_ptr<TrajectoryVariables> &state_vars,
    const int state_len,
    const std::shared_ptr<TrajectoryVariables> &ctrl_vars,
    const int control_len,
    const double dt_segment,
    const DynFn &dyn_fn,
    const JacobianDynFn &jac_dyn_wrt_state_fn,
    const JacobianDynFn &jac_dyn_wrt_control_fn)
    : ConstraintSet(num_constraints, "trap_col_constraints")
    , m_state_vars{state_vars}
    , m_state_len{state_len}
    , m_ctrl_vars{ctrl_vars}
    , m_control_len{control_len}
    , m_dt_segment{dt_segment}
    , m_dyn_fn{dyn_fn}
    , m_jac_dyn_wrt_state_fn{jac_dyn_wrt_state_fn}
    , m_jac_dyn_wrt_control_fn{jac_dyn_wrt_control_fn}
{
    const Eigen::VectorXd state_vec = m_state_vars->GetValues();
    assert(state_vec.size() % m_state_len == 0);
    const int num_knot_pts = state_vec.size() / m_state_len;
    m_num_segments = num_knot_pts - 1;
    assert(num_constraints == m_num_segments * m_state_len);
}

Eigen::VectorXd TrapezoidalCollocationConstraints::GetValues() const
{
    const Eigen::VectorXd state_vec = m_state_vars->GetValues();
    const Eigen::VectorXd ctrl_vec = m_ctrl_vars->GetValues();

    assert(state_vec.size() % m_state_len == 0);
    assert(ctrl_vec.size() % m_control_len == 0);
    assert(ctrl_vec.size() / m_control_len == state_vec.size() / m_state_len);

    // fill in defect constraint values
    Eigen::VectorXd defect_constraints = Eigen::VectorXd::Zero(GetRows());

    // k represents the kth vector constraint equation (defect). The number
    // of vector constraint (defect) equations equals the number of time
    // segments, which is one less than the number of time points.
    for (int k{}; k < m_num_segments; ++k) {
        // get state and control k
        auto state_view_k
            = state_vec(Eigen::seqN(k * m_state_len, m_state_len));
        auto control_view_k
            = ctrl_vec(Eigen::seqN(k * m_control_len, m_control_len));
        // time relative to start time of zero
        const double tk = k * m_dt_segment;
        const Eigen::VectorXd fk = m_dyn_fn(state_view_k, control_view_k, tk);
        assert(fk.size() == m_state_len);

        // get state and control k+1. The number of time points is one more
        // than the number of time segements, so this index should not go
        // out of bounds.
        auto state_view_k1
            = state_vec(Eigen::seqN((k + 1) * m_state_len, m_state_len));
        auto control_view_k1
            = ctrl_vec(Eigen::seqN((k + 1) * m_control_len, m_control_len));
        // time relative to start time of zero
        const double tk1 = (k + 1) * m_dt_segment;
        const Eigen::VectorXd fk1
            = m_dyn_fn(state_view_k1, control_view_k1, tk1);
        assert(fk1.size() == m_state_len);

        // calculate vector of defect k (for vector based defects) and set
        // them in final combined constraints vector
        const Eigen::VectorXd defectk
            = state_view_k1 - state_view_k - m_dt_segment / 2.0 * (fk + fk1);
        defect_constraints(Eigen::seqN(k * m_state_len, m_state_len)) = defectk;
        // std::cout << "k = " << k << std::endl;
        // std::cout << "fk = \n" << fk << std::endl;
        // std::cout << "fk1 = \n" << fk1 << std::endl;
    }

    return defect_constraints;
}

void TrapezoidalCollocationConstraints::FillJacobianBlock(
    std::string var_set,
    ifopt::Component::Jacobian &jac_block) const
{
    if (var_set == m_state_vars->GetName()) {
        FillJacobianWrt(VariableType::STATE, jac_block);
    } else if (var_set == m_ctrl_vars->GetName()) {
        FillJacobianWrt(VariableType::CONTROL, jac_block);
    }
}

int TrapezoidalCollocationConstraints::getVarTypeLen(
    const VariableType var_type) const
{
    switch (var_type) {
        case VariableType::STATE:
            return m_state_len;

        case VariableType::CONTROL:
            return m_control_len;
    }
    assert(false);
    return m_state_len;
}

void TrapezoidalCollocationConstraints::FillJacobianWrt(
    const VariableType var_type,
    ifopt::Component::Jacobian &jac_block) const
{
    const Eigen::VectorXd state_vec = m_state_vars->GetValues();
    const Eigen::VectorXd ctrl_vec = m_ctrl_vars->GetValues();

    // use list of triplets to simplify and avoid costly random
    // insertions when constructing the final sparse jacobian matrix
    std::vector<Eigen::Triplet<double>> triplet_list;
    const int var_type_len = getVarTypeLen(var_type);
    const int num_nonzero_submatrices = 2;
    const int num_defect_vec_eqns = m_num_segments;
    triplet_list.reserve(m_state_len * var_type_len * num_nonzero_submatrices
                         * num_defect_vec_eqns);

    // Here k represents the kth vector defect constraint equation. Set the stop
    // point such that the state at time point j=k+1 can be accessed for the
    // last iteration.
    const int k_max = m_num_segments;

    // The jacobian of defect k w.r.t control vector j is nonzero for j=k and
    // j=k+1 (gives two non-zero submatrices in the output jacobian). This
    // submatrix starts at (k*state_len, j*control_len) and has
    // size=(control_len x state_len).
    for (size_t k{}; k < k_max; ++k) {
        for (size_t j = k; j < k + 2; ++j) {
            // get state, control, and time at time index j
            auto statej = state_vec(Eigen::seqN(j * m_state_len, m_state_len));
            auto controlj
                = ctrl_vec(Eigen::seqN(j * m_control_len, m_control_len));
            const double tj = m_dt_segment * j;
            // defects increment for each row
            const int row_start = k * m_state_len;
            // control/state vectors increment for each column
            const int col_start = j * var_type_len;

            // get triplets for jacobian of defect k w.r.t state or control
            // vector at time point j
            ifopt::Component::Jacobian jac_constraints_wrt_var
                = jacConstraintsWrtVar(var_type, k, j, statej, controlj, tj);

            // extract triplets for final jacobian construction and
            // making sure to offset the indices
            auto sub_triplets = sparseMatrixToTriplets(jac_constraints_wrt_var,
                                                       row_start,
                                                       col_start);
            triplet_list.insert(triplet_list.cend(),
                                sub_triplets.cbegin(),
                                sub_triplets.cend());
        }
    }

    jac_block.setFromTriplets(triplet_list.cbegin(), triplet_list.cend());
}

ifopt::Component::Jacobian
    TrapezoidalCollocationConstraints::jacConstraintsWrtVar(
        const VariableType var_type,
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const
{
    switch (var_type) {
        case VariableType::STATE:
            return jacConstraintsWrtState(k, j, state, control, time);

        case VariableType::CONTROL:
            return jacConstraintsWrtControl(k, j, state, control, time);
    }
    assert(false);
    return jacConstraintsWrtState(k, j, state, control, time);
}

ifopt::Component::Jacobian
    TrapezoidalCollocationConstraints::jacConstraintsWrtState(
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &statej,
        const Eigen::VectorXd &controlj,
        const double timej) const
{
    // In general the jacobian of defect k w.r.t state j is:
    // dck_dxj = dxk1_dxj - dxk_dxj - hk/2*(dfk1_dxj + dfk_dxj)

    // j=k => dxk1_dxj=0 and dfk1_dxj=0
    // j=k+1 => dxk_dxj=0 and dfk_dxj=0

    // jacobians of dynamics. This represents either dfk_dxj
    // (for j=k) or dfk1_dxj (for j=k+1) to reduce duplicate
    // code.
    ifopt::Component::Jacobian dfj_dxj
        = m_jac_dyn_wrt_state_fn(statej, controlj, timej);
    // jacobian of discrete state. This represents either
    // dxk_dxj (for j=k) or dxk1_dxj (for j=k+1) to reduce
    // duplicate code.
    ifopt::Component::Jacobian dxk_dxj(m_state_len, m_state_len);
    dxk_dxj.setIdentity();

    // jacobian of defect k w.r.t state j.
    const auto hk = m_dt_segment;
    ifopt::Component::Jacobian dck_dxj = -hk / 2 * dfj_dxj;
    if (k == j) {
        dck_dxj -= dxk_dxj;
    } else {
        dck_dxj += dxk_dxj;
    }

    return dck_dxj;
}

ifopt::Component::Jacobian
    TrapezoidalCollocationConstraints::jacConstraintsWrtControl(
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &statej,
        const Eigen::VectorXd &controlj,
        const double timej) const
{
    // In general the jacobian of defect k w.r.t control j is:
    // dck_duj = - hk/2*(dfk1_duj + dfk_duj)

    // j=k => dfk1_duj=0
    // j=k+1 => dfk_duj=0

    // jacobians of dynamics. This represents either dfk_duj
    // (for j=k) or dfk1_duj (for j=k+1) to reduce duplicate
    // code.
    ifopt::Component::Jacobian dfj_duj
        = m_jac_dyn_wrt_control_fn(statej, controlj, timej);

    const auto hk = m_dt_segment;
    return -hk / 2 * dfj_duj;
}
