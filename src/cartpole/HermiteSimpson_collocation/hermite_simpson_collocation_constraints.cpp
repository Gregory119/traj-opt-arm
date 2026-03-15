#include "HermiteSimpson_collocation_constraints.hpp"

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

HermSimpCollocationConstraints::HermSimpCollocationConstraints(
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
    const JacobianDynFn &jac_dyn_wrt_control_fn)
    : ConstraintSet(num_constraints, "HS_col_constraints")
    , m_state_vars{state_vars}
    , m_state_len{state_len}
    , m_ctrl_vars{ctrl_vars}
    , m_control_len{control_len}
    , m_state_mid_vars{state_mid_vars}
    , m_ctrl_mid_vars{ctrl_mid_vars}
    , m_dt_segment{dt_segment}
    , m_dyn_fn{dyn_fn}
    , m_jac_dyn_wrt_state_fn{jac_dyn_wrt_state_fn}
    , m_jac_dyn_wrt_control_fn{jac_dyn_wrt_control_fn}
{
    assert(num_constraints % (2 * m_state_len) == 0);
    m_num_segments = num_constraints / (2 * m_state_len);
}

Eigen::VectorXd HermSimpCollocationConstraints::GetValues() const
{
    const Eigen::VectorXd state_vars = m_state_vars->GetValues();
    const Eigen::VectorXd control_vars = m_ctrl_vars->GetValues();

    // midpoints
    const Eigen::VectorXd state_mid_vars = m_state_mid_vars->GetValues();
    const Eigen::VectorXd control_mid_vars = m_ctrl_mid_vars->GetValues();

    assert(state_vars.size() % m_state_len == 0);
    assert(control_vars.size() % m_control_len == 0);
    assert(state_mid_vars.size() % m_state_len == 0);
    assert(control_mid_vars.size() % m_control_len == 0);


    assert(control_vars.size() / m_control_len
           == state_vars.size() / m_state_len);

    // fill in defect constraint values
    Eigen::VectorXd defect_constraints = Eigen::VectorXd::Zero(GetRows());

    // k indexes a trajectory segment
    // each segment contributes two vector constraints of length m_state_len: 
    // the Hermite midpoint constraint c_mid and the Simpson defect constraint c_def

    assert(state_mid_vars.size() / m_state_len == m_num_segments);
    assert(control_mid_vars.size() / m_control_len == m_num_segments);

    const double h = m_dt_segment;

    for (int k{}; k < m_num_segments; ++k) {
        // get state and control k
        auto xk 
            = state_vars(Eigen::seqN(k * m_state_len, m_state_len));
        auto uk
            = control_vars(Eigen::seqN(k * m_control_len, m_control_len));
        // time relative to start time of zero
        const double tk = k * m_dt_segment;
        const Eigen::VectorXd fk = m_dyn_fn(xk, uk, tk);
        assert(fk.size() == m_state_len);

        // get state and control k+1. The number of time points is one more
        // than the number of time segements, so this index should not go
        // out of bounds.
        auto xk1 
            = state_vars(Eigen::seqN((k + 1) * m_state_len, m_state_len));
        auto uk1 
            = control_vars(Eigen::seqN((k + 1) * m_control_len, m_control_len));
        // time relative to start time of zero
        const double tk1 = (k + 1) * m_dt_segment;
        const Eigen::VectorXd fk1
            = m_dyn_fn(xk1, uk1, tk1);
        assert(fk1.size() == m_state_len);


        // midpoint decision vars
        auto xc = state_mid_vars(Eigen::seqN(k * m_state_len, m_state_len));
        auto uc = control_mid_vars(Eigen::seqN(k * m_control_len, m_control_len));
        // midpoint time
        const double tc = (k + 0.5) * h;
        const Eigen::VectorXd fc = m_dyn_fn(xc, uc, tc);

        //Hermite interpolation
        const Eigen::VectorXd c_mid = xc - 0.5 * (xk + xk1) - (h / 8.0) * (fk - fk1);
        //Simpson defect
        const Eigen::VectorXd c_def = xk1 - xk - (h / 6.0) * (fk + 4.0 * fc + fk1);


        const int row_mid = (2 * k) * m_state_len;
        const int row_def = row_mid + m_state_len;
        defect_constraints(Eigen::seqN(row_mid, m_state_len)) = c_mid;
        defect_constraints(Eigen::seqN(row_def, m_state_len)) = c_def;
    }

    return defect_constraints;
}

void HermSimpCollocationConstraints::FillJacobianBlock(
    std::string var_set,
    ifopt::Component::Jacobian &jac_block) const
{
    if (var_set == m_state_vars->GetName()) {
        FillJacobianWrt(VariableType::STATE, jac_block);
    } else if (var_set == m_ctrl_vars->GetName()) {
        FillJacobianWrt(VariableType::CONTROL, jac_block);
    } else if (var_set == m_state_mid_vars->GetName()) {
        FillJacobianWrt(VariableType::STATE_MID, jac_block);
    } else if (var_set == m_ctrl_mid_vars->GetName()) {
        FillJacobianWrt(VariableType::CONTROL_MID, jac_block);
    }

}

int HermSimpCollocationConstraints::getVarTypeLen(
    const VariableType var_type) const
{
    switch (var_type) {
        case VariableType::STATE:
        case VariableType::STATE_MID:
            return m_state_len;

        case VariableType::CONTROL:
        case VariableType::CONTROL_MID:
            return m_control_len;
    }
    assert(false);
    return m_state_len;
}

void HermSimpCollocationConstraints::FillJacobianWrt(
    const VariableType var_type,
    ifopt::Component::Jacobian &jac_block) const
{
    const Eigen::VectorXd state_vars = m_state_vars->GetValues();
    const Eigen::VectorXd control_vars = m_ctrl_vars->GetValues();

    const Eigen::VectorXd state_mid_vars = m_state_mid_vars->GetValues();
    const Eigen::VectorXd control_mid_vars = m_ctrl_mid_vars->GetValues();

    const double h = m_dt_segment; //will probably just rename m_dt_segment later

    // use list of triplets to simplify and avoid costly random
    // insertions when constructing the final sparse jacobian matrix
    std::vector<Eigen::Triplet<double>> triplet_list;
    const int var_type_len = getVarTypeLen(var_type);
    const int num_nonzero_submatrices = 2;
    const int num_defect_vec_eqns = m_num_segments;
    triplet_list.reserve(m_state_len * var_type_len * num_nonzero_submatrices
                         * num_defect_vec_eqns);

    // k indexes a trajectory segment, and j indexes the neighboring knot block
    // used for that segment (j = k or j = k+1)
    // For each segment, the local Jacobian contribution is the stacked block
    // for [c_mid; c_def] so the row offset is (2*k)*m_state_len
    // knot variables use column block j
    // midpoint variables use column block k
    const int k_max = m_num_segments;

    for (size_t k{}; k < k_max; ++k) {
        for (size_t j = k; j < k + 2; ++j) {

            const bool is_mid = (var_type == VariableType::STATE_MID
                     || var_type == VariableType::CONTROL_MID);
            // for midpoint var sets, there is only one variable block per segment index k not per knot index j
            // for knot var sets use index j
            const size_t col_idx = is_mid ? k : j;

            auto statej = is_mid ? state_mid_vars(Eigen::seqN(k * m_state_len, m_state_len))
                     : state_vars(Eigen::seqN(j * m_state_len, m_state_len));
            auto controlj = is_mid ? control_mid_vars(Eigen::seqN(k * m_control_len, m_control_len))
                       : control_vars(Eigen::seqN(j * m_control_len, m_control_len));

            const double tj = is_mid ? ((static_cast<double>(k) + 0.5) * h)
                         : (m_dt_segment * j);

            // defects increment for each row
            const int row_start = (2*k) * m_state_len;
            // control/state vectors increment for each column
            const int col_start = static_cast<int>(col_idx) * var_type_len;

            // get triplets for the segment constraint Jacobian
            // [c_mid; c_def] w.r.t. the selected variable block
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
    HermSimpCollocationConstraints::jacConstraintsWrtVar(
        const VariableType var_type,
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const
{
    const double h = m_dt_segment;
    switch (var_type) {
        case VariableType::STATE:
            return jacConstraintsWrtState(k, j, state, control, time);

        case VariableType::CONTROL:
            return jacConstraintsWrtControl(k, j, state, control, time);

        case VariableType::STATE_MID: {
            // for mdpoint vars only j==k matter
            if (j != k) {
                ifopt::Component::Jacobian Z(2 * m_state_len, m_state_len);
                Z.setZero();
                return Z;
            }

            // top block is identity
            ifopt::Component::Jacobian I(m_state_len, m_state_len);
            I.setIdentity();

            // bottom blocxk -(2h/3) * dfc/dxc
            ifopt::Component::Jacobian Ac = m_jac_dyn_wrt_state_fn(state, control, time);
            ifopt::Component::Jacobian dcd_dxc = -(2.0 * h / 3.0) * Ac;

            std::vector<Eigen::Triplet<double>> trips;
            auto t1 = sparseMatrixToTriplets(I, 0, 0);
            auto t2 = sparseMatrixToTriplets(dcd_dxc, m_state_len, 0);
            trips.insert(trips.end(), t1.begin(), t1.end());
            trips.insert(trips.end(), t2.begin(), t2.end());

            ifopt::Component::Jacobian J(2 * m_state_len, m_state_len);
            J.setFromTriplets(trips.begin(), trips.end());
            return J;
        }

        case VariableType::CONTROL_MID: {
            if (j != k) {
                ifopt::Component::Jacobian Z(2 * m_state_len, m_control_len);
                Z.setZero();
                return Z;
            }
            // d(c_mid)/d(uc) = 0
            // d(c_def)/d(uc) = -(2h/3) * dfc/duc
            ifopt::Component::Jacobian Bc = m_jac_dyn_wrt_control_fn(state, control, time);
            ifopt::Component::Jacobian dcd_duc = -(2.0 * h / 3.0) * Bc;

            auto t = sparseMatrixToTriplets(dcd_duc, m_state_len, 0);

            ifopt::Component::Jacobian J(2 * m_state_len, m_control_len);
            J.setFromTriplets(t.begin(), t.end());
            return J;
        }
    }
    assert(false);
    ifopt::Component::Jacobian Z(2 * m_state_len, m_state_len);
    Z.setZero();
    return Z;
}

ifopt::Component::Jacobian
    HermSimpCollocationConstraints::jacConstraintsWrtState(
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const
{
    const double h = m_dt_segment;

    // Only endpoints j==k or j==k+1 are valid for knot state derivatives
    if (!(j == k || j == k + 1)) {
        ifopt::Component::Jacobian Z(2 * m_state_len, m_state_len);
        Z.setZero();
        return Z;
    }

    const bool left = (j == k);

    // A = df/dx at the evaluation point either k or k+1 depending on j
    ifopt::Component::Jacobian A = m_jac_dyn_wrt_state_fn(state, control, time);

    ifopt::Component::Jacobian I(m_state_len, m_state_len);
    I.setIdentity();

    // c_mid = xc - 0.5(xk+xk1) - h/8(fk - fk1)
    // c_def = xk1 - xk - h/6(fk + 4fc + fk1)
    const double s_mid = left ? -1.0 : +1.0; // fk term sign in c_mid
    const double s_def_I = left ? -1.0 : +1.0; // xk vs xk1 sign in c_def

    ifopt::Component::Jacobian dcm_dx = (-0.5) * I + (s_mid * (h / 8.0)) * A;
    ifopt::Component::Jacobian dcd_dx = (s_def_I) * I + (-(h / 6.0)) * A;

    std::vector<Eigen::Triplet<double>> trips;
    auto t1 = sparseMatrixToTriplets(dcm_dx, 0, 0);
    auto t2 = sparseMatrixToTriplets(dcd_dx, m_state_len, 0);
    trips.insert(trips.end(), t1.begin(), t1.end());
    trips.insert(trips.end(), t2.begin(), t2.end());

    ifopt::Component::Jacobian J(2 * m_state_len, m_state_len);
    J.setFromTriplets(trips.begin(), trips.end());
    return J;
}

ifopt::Component::Jacobian
    HermSimpCollocationConstraints::jacConstraintsWrtControl(
        const size_t k,
        const size_t j,
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        const double time) const
{
    const double h = m_dt_segment;

    // Only endpoints j==k or j==k+1 are valid for knot control derivatives
    if (!(j == k || j == k + 1)) {
        ifopt::Component::Jacobian Z(2 * m_state_len, m_control_len);
        Z.setZero();
        return Z;
    }

    const bool left = (j == k);

    // B = df/du at the evaluation point either k or k+1 depending on j
    ifopt::Component::Jacobian B = m_jac_dyn_wrt_control_fn(state, control, time);

    // HS blocks
    // d(c_mid)/d(uk)  = -(h/8) B0,   d(c_mid)/d(uk1) = +(h/8) B1
    // d(c_def)/d(uk)  = -(h/6) B0,   d(c_def)/d(uk1) = -(h/6) B1
    const double s_mid = left ? -1.0 : +1.0;

    ifopt::Component::Jacobian dcm_du = (s_mid * (h / 8.0)) * B;
    ifopt::Component::Jacobian dcd_du = (-(h / 6.0)) * B;

    std::vector<Eigen::Triplet<double>> trips;
    auto t1 = sparseMatrixToTriplets(dcm_du, 0, 0);
    auto t2 = sparseMatrixToTriplets(dcd_du, m_state_len, 0);
    trips.insert(trips.end(), t1.begin(), t1.end());
    trips.insert(trips.end(), t2.begin(), t2.end());

    ifopt::Component::Jacobian J(2 * m_state_len, m_control_len);
    J.setFromTriplets(trips.begin(), trips.end());
    return J;
}
