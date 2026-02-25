#include "trapezoidal_collocation_contraints.hpp"

std::vector<Eigen::Triplet<double>> sparseMatrixToTriplets(
    const ifopt::Component::Jacobian &mat,
    const int row_start,
    const int col_start)
{
    std::vector<Eigen::Triplet<Scalar>> triplets;
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
    const std::string &state_var_set_name,
    const int state_len,
    const std::string &control_var_set_name,
    const int control_len,
    const double segment_dt,
    const DynFn &dyn_fn,
    const JacobianDynFn &jac_dyn_fn_wrt_state)
    : ConstraintSet(num_constraints, "trap_col_constraints")
    , m_state_var_set_name{state_var_set_name}
    , m_state_len{state_len}
    , m_control_var_set_name{control_var_set_name}
    , m_control_len{control_len}
    , m_segment_dt{segment_dt}
    , m_dyn_fn{dyn_fn}
    , m_jac_dyn_fn_wrt_state{jac_dyn_fn_wrt_state}
{
    const Eigen::VectorXd state_vars
        = GetVariables()->GetComponent(m_state_var_set_name)->GetValues();
    assert(state_var.size() % m_state_len == 0);
    const int num_knot_pts = state_var.size() / m_state_len;
    m_num_segments = num_knot_pts - 1;
    assert(num_constraints == num_segments * m_state_len);
}

Eigen::VectorXd TrapezoidalCollocationConstraints::GetValues() const
{
    const Eigen::VectorXd state_vars
        = GetVariables()->GetComponent(m_state_var_set_name)->GetValues();
    const Eigen::VectorXd control_vars
        = GetVariables()->GetComponent(m_control_var_set_name)->GetValues();

    assert(state_vars.size() % m_state_len == 0);
    assert(control_vars.size() % m_control_len == 0);
    assert(control_vars.size() / m_control_len
           == state_vars.size() / m_state_len);

    // fill in defect constraint values
    auto defect_constraints = Eigen::VectorXd::Zero(GetRows());

    // k represents the kth vector constraint equation (defect). The number
    // of vector constraint (defect) equations equals the number of time
    // segments, which is one less than the number of time points.
    for (int k{}; k < m_num_segments; ++k) {
        // get state and control k
        auto state_view_k
            = state_vars(Eigen::seqN(k * m_state_len, m_state_len));
        auto control_view_k
            = control_vars(Eigen::seqN(k * m_control_len, m_control_len));
        // time relative to start time of zero
        const double tk = k * m_segment_dt;
        const Eigen::VectorXd fk = m_dyn_fn(state_view_k, control_view_k, tk);
        assert(fk.size() == m_state_len);

        // get state and control k+1. The number of time points is one more
        // than the number of time segements, so this index should not go
        // out of bounds.
        auto state_view_k1
            = state_vars(Eigen::seqN((k + 1) * m_state_len, m_state_len));
        auto control_view_k1
            = control_vars(Eigen::seqN((k + 1) * m_control_len, m_control_len));
        // time relative to start time of zero
        const double tk1 = (k + 1) * m_segment_dt;
        const Eigen::VectorXd fk1
            = m_dyn_fn(state_view_k1, control_view_k1, tk1);
        assert(fk1.size() == m_state_len);

        // calculate vector of defect k (for vector based defects) and set
        // them in final combined constraints vector
        const Eigen::VectorXd defectk
            = state_view_k1 - state_view_k - m_segment_dt / 2.0 * (fk + fk1);
        defect_constraints(Eigen::seqN(k * m_state_len, m_state_len)) = defectk;
    }

    return defect_constraints;
}

void TrapezoidalCollocationConstraints::FillJacobianWrtState(
    ifopt::Component::Jacobian &jac_block) const
{
    const Eigen::VectorXd state_vars
        = GetVariables()->GetComponent(var_set)->GetValues();
    const Eigen::VectorXd control_vars
        = GetVariables()->GetComponent(m_control_var_set_name)->GetValues();

    // use list of triplets to simplify and avoid costly random
    // insertions when constructing the final sparse jacobian matrix
    std::vector<Eigen::Triplet<double>> triplet_list;
    triplet_list.reserve(m_state_len * m_state_len * 2 * m_num_segments);

    // Here k represents the kth defect constraint equation. Set the
    // stop point such that the state at time point j=k+1 can be
    // accessed for the last iteration.
    const int k_max = m_num_segments;

    // The jacobian of defect k w.r.t state vector j is nonzero for
    // j=k and j=k+1. This submatrix starts at (k*state_len,
    // j*state_len) and has size=state_len x state_len.
    for (size_t k{}; k < k_max; ++k) {
        for (size_t j = k; j < k + 2; ++j) {
            // get state, control, and time at time index j
            auto statej = state_vars(Eigen::seqN(j * m_state_len, m_state_len));
            auto controlj
                = control_vars(Eigen::seqN(j * m_control_len, m_control_len));
            const double tj = m_segment_dt * j;
            // defects increment for each row
            const int row_start = k * m_state_len;
            // state vectors increment for each column
            const int col_start = j * m_state_len;

            // In general the jacobian of defect k w.r.t state j is:
            // dck_dxj = dxk1_dxj - dxk_dxj - hk/2*(dfk1_dxj + dfk_dxj)

            // j=k => dxk1_dxj=0 and dfk1_dxj=0
            // j=k+1 => dxk_dxj=0 and dfk_dxj=0

            // jacobians of dynamics. This represents either dfk_dxj
            // (for j=k) or dfk1_dxj (for j=k+1) to reduce duplicate
            // code.
            ifopt::Component::Jacobian dfk_dxj
                = m_jac_dyn_fn_wrt_state(statej, controlj, tj);
            // jacobian of discrete state. This represents either
            // dxk_dxj (for j=k) or dxk1_dxj (for j=k+1) to reduce
            // duplicate code.
            auto dxk_dxj = ifopt::Component::Jacobian::Identity(m_state_len,
                                                                m_state_len);

            // jacobian of defect k w.r.t state j.
            auto dck_dxj = -hk / 2 * dfk_dxj;
            if (k == j) {
                dck_dxj -= dxk_dxj;
            } else {
                dck_dxj += dxk_dxj;
            }
            // extract triplets for final jacobian construction and
            // making sure to offset the indices
            auto sub_triplets
                = sparseMatrixToTriplets(dck_dxj, row_start, col_start);
            triplet_list.insert(triplet_list.cend(),
                                sub_triplets.cbegin(),
                                sub_triplets.cend());
        }
    }

    jac_block.setFromTriplets(triplet_list.cbegin(), triplet_list.cend());
}
