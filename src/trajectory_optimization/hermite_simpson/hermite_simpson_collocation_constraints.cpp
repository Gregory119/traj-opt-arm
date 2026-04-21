#include "hermite_simpson_collocation_constraints.hpp"

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

HermiteMidpointConstraints::HermiteMidpointConstraints(
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
    : ConstraintSet(num_constraints, "Hermite_midpoint_constraints")
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
    assert(num_constraints % m_state_len == 0);
    m_num_segments = num_constraints / m_state_len;
}

Eigen::VectorXd HermiteMidpointConstraints::GetValues() const
{
    const Eigen::VectorXd state_vars = m_state_vars->GetValues();
    const Eigen::VectorXd control_vars = m_ctrl_vars->GetValues();
    const Eigen::VectorXd state_mid_vars = m_state_mid_vars->GetValues();
    const Eigen::VectorXd control_mid_vars = m_ctrl_mid_vars->GetValues();

    assert(state_vars.size() % m_state_len == 0);
    assert(control_vars.size() % m_control_len == 0);
    assert(state_mid_vars.size() % m_state_len == 0);
    assert(control_mid_vars.size() % m_control_len == 0);
    assert(control_vars.size() / m_control_len
           == state_vars.size() / m_state_len);
    assert(state_mid_vars.size() / m_state_len == m_num_segments);
    assert(control_mid_vars.size() / m_control_len == m_num_segments);

    Eigen::VectorXd constraint_values = Eigen::VectorXd::Zero(GetRows());
    const double h = m_dt_segment;

    for (int k{}; k < m_num_segments; ++k) {
        const auto xk = state_vars(Eigen::seqN(k * m_state_len, m_state_len));
        const auto uk
            = control_vars(Eigen::seqN(k * m_control_len, m_control_len));
        const double tk = k * m_dt_segment;
        const Eigen::VectorXd fk = m_dyn_fn(xk, uk, tk);
        assert(fk.size() == m_state_len);

        const auto xk1
            = state_vars(Eigen::seqN((k + 1) * m_state_len, m_state_len));
        const auto uk1
            = control_vars(Eigen::seqN((k + 1) * m_control_len, m_control_len));
        const double tk1 = (k + 1) * m_dt_segment;
        const Eigen::VectorXd fk1 = m_dyn_fn(xk1, uk1, tk1);
        assert(fk1.size() == m_state_len);

        const auto xc
            = state_mid_vars(Eigen::seqN(k * m_state_len, m_state_len));

        const Eigen::VectorXd c_mid
            = xc - 0.5 * (xk + xk1) - (h / 8.0) * (fk - fk1);

        const int row = k * m_state_len;
        constraint_values(Eigen::seqN(row, m_state_len)) = c_mid;
    }

    return constraint_values;
}

void HermiteMidpointConstraints::FillJacobianBlock(
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

int HermiteMidpointConstraints::getVarTypeLen(const VariableType var_type) const
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

void HermiteMidpointConstraints::FillJacobianWrt(
    const VariableType var_type,
    ifopt::Component::Jacobian &jac_block) const
{
    const Eigen::VectorXd state_vars = m_state_vars->GetValues();
    const Eigen::VectorXd control_vars = m_ctrl_vars->GetValues();

    const Eigen::VectorXd state_mid_vars = m_state_mid_vars->GetValues();
    const Eigen::VectorXd control_mid_vars = m_ctrl_mid_vars->GetValues();

    const double h
        = m_dt_segment;  // will probably just rename m_dt_segment later

    // use list of triplets to simplify and avoid costly random
    // insertions when constructing the final sparse jacobian matrix.
    std::vector<Eigen::Triplet<double>> triplet_list;
    const int var_type_len = getVarTypeLen(var_type);
    const int num_nonzero_submatrices = 2;
    const int num_defect_vec_eqns = m_num_segments;
    triplet_list.reserve(m_state_len * var_type_len * num_nonzero_submatrices
                         * num_defect_vec_eqns);

    // k indexes a trajectory segment,
    // j indexes the variable block whose contribution is being inserted into
    // the full jacobian for segment k.
    //
    // for knot variable sets segment k depends on both
    // endpoint knot blocks so use j = k and j = k + 1.
    //
    // for midpoint variable sets there is only one
    // variable block associated with segment k so only use j = k.
    //
    // this is why j_max is k + 2 for knot variables and k + 1 for midpoint
    // variables.
    const size_t k_max = static_cast<size_t>(m_num_segments);

    for (size_t k{}; k < k_max; ++k) {
        const bool is_mid = (var_type == VariableType::STATE_MID
                             || var_type == VariableType::CONTROL_MID);

        // midpoint variable sets have one block per segment so only j = k is
        // vald.
        // knot variable sets have two neighboring blocks for
        // segment k, j = k an j = k + 1.
        const size_t j_max = is_mid ? (k + 1) : (k + 2);

        for (size_t j = k; j < j_max; ++j) {
            // j is used here instead of k because the loop bounds above
            // guarantee that j already refers to the correct variable block
            //
            // midpoint  j = k is usd.
            //
            // knot j = k and j = k + 1 are used.
            auto statej
                = is_mid
                      ? state_mid_vars(
                            Eigen::seqN(j * m_state_len, m_state_len))
                      : state_vars(Eigen::seqN(j * m_state_len, m_state_len));
            auto controlj
                = is_mid ? control_mid_vars(
                               Eigen::seqN(j * m_control_len, m_control_len))
                         : control_vars(
                               Eigen::seqN(j * m_control_len, m_control_len));

            const double tj = is_mid ? ((static_cast<double>(j) + 0.5) * h)
                                     : (m_dt_segment * j);

            // segment k has rows starting at k * m_state_len
            const int row_start = static_cast<int>(k) * m_state_len;

            // variable block j has columns starting at j * var_type_len
            const int col_start = static_cast<int>(j) * var_type_len;

            ifopt::Component::Jacobian jac_constraints_wrt_var
                = jacConstraintsWrtVar(var_type, k, j, statej, controlj, tj);

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

ifopt::Component::Jacobian HermiteMidpointConstraints::jacConstraintsWrtVar(
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

        case VariableType::STATE_MID: {
            if (j != k) {
                ifopt::Component::Jacobian Z(m_state_len, m_state_len);
                Z.setZero();
                return Z;
            }

            ifopt::Component::Jacobian I(m_state_len, m_state_len);
            I.setIdentity();
            return I;
        }

        case VariableType::CONTROL_MID: {
            if (j != k) {
                ifopt::Component::Jacobian Z(m_state_len, m_control_len);
                Z.setZero();
                return Z;
            }

            ifopt::Component::Jacobian Z(m_state_len, m_control_len);
            Z.setZero();
            return Z;
        }
    }

    assert(false);
    ifopt::Component::Jacobian Z(m_state_len, m_state_len);
    Z.setZero();
    return Z;
}

ifopt::Component::Jacobian HermiteMidpointConstraints::jacConstraintsWrtState(
    const size_t k,
    const size_t j,
    const Eigen::VectorXd &state,
    const Eigen::VectorXd &control,
    const double time) const
{
    const double h = m_dt_segment;

    if (!(j == k || j == k + 1)) {
        ifopt::Component::Jacobian Z(m_state_len, m_state_len);
        Z.setZero();
        return Z;
    }

    const bool left = (j == k);
    ifopt::Component::Jacobian dfj_dxj
        = m_jac_dyn_wrt_state_fn(state, control, time);

    ifopt::Component::Jacobian I(m_state_len, m_state_len);
    I.setIdentity();

    const double s_mid = left ? -1.0 : +1.0;

    ifopt::Component::Jacobian dcm_dx
        = (-0.5) * I + (s_mid * (h / 8.0)) * dfj_dxj;

    return dcm_dx;
}

ifopt::Component::Jacobian HermiteMidpointConstraints::jacConstraintsWrtControl(
    const size_t k,
    const size_t j,
    const Eigen::VectorXd &state,
    const Eigen::VectorXd &control,
    const double time) const
{
    const double h = m_dt_segment;

    if (!(j == k || j == k + 1)) {
        ifopt::Component::Jacobian Z(m_state_len, m_control_len);
        Z.setZero();
        return Z;
    }

    const bool left = (j == k);
    ifopt::Component::Jacobian B
        = m_jac_dyn_wrt_control_fn(state, control, time);

    const double s_mid = left ? -1.0 : +1.0;

    ifopt::Component::Jacobian dcm_du = (s_mid * (h / 8.0)) * B;
    return dcm_du;
}

/////////////////////////////////////////////////////////////
// SimpsonDefectConstraints
/////////////////////////////////////////////////////////////

SimpsonDefectConstraints::SimpsonDefectConstraints(
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
    : ConstraintSet(num_constraints, "simpson_defect_constraints")
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
    assert(num_constraints % m_state_len == 0);
    m_num_segments = num_constraints / m_state_len;
}

Eigen::VectorXd SimpsonDefectConstraints::GetValues() const
{
    const Eigen::VectorXd state_vars = m_state_vars->GetValues();
    const Eigen::VectorXd control_vars = m_ctrl_vars->GetValues();
    const Eigen::VectorXd state_mid_vars = m_state_mid_vars->GetValues();
    const Eigen::VectorXd control_mid_vars = m_ctrl_mid_vars->GetValues();

    assert(state_vars.size() % m_state_len == 0);
    assert(control_vars.size() % m_control_len == 0);
    assert(state_mid_vars.size() % m_state_len == 0);
    assert(control_mid_vars.size() % m_control_len == 0);
    assert(control_vars.size() / m_control_len
           == state_vars.size() / m_state_len);
    assert(state_mid_vars.size() / m_state_len == m_num_segments);
    assert(control_mid_vars.size() / m_control_len == m_num_segments);

    Eigen::VectorXd constraint_values = Eigen::VectorXd::Zero(GetRows());
    const double h = m_dt_segment;

    for (int k{}; k < m_num_segments; ++k) {
        const auto xk = state_vars(Eigen::seqN(k * m_state_len, m_state_len));
        const auto uk
            = control_vars(Eigen::seqN(k * m_control_len, m_control_len));
        const double tk = k * m_dt_segment;
        const Eigen::VectorXd fk = m_dyn_fn(xk, uk, tk);
        assert(fk.size() == m_state_len);

        const auto xk1
            = state_vars(Eigen::seqN((k + 1) * m_state_len, m_state_len));
        const auto uk1
            = control_vars(Eigen::seqN((k + 1) * m_control_len, m_control_len));
        const double tk1 = (k + 1) * m_dt_segment;
        const Eigen::VectorXd fk1 = m_dyn_fn(xk1, uk1, tk1);
        assert(fk1.size() == m_state_len);

        const auto xc
            = state_mid_vars(Eigen::seqN(k * m_state_len, m_state_len));
        const auto uc
            = control_mid_vars(Eigen::seqN(k * m_control_len, m_control_len));
        const double tc = (k + 0.5) * h;
        const Eigen::VectorXd fc = m_dyn_fn(xc, uc, tc);

        const Eigen::VectorXd c_def
            = xk1 - xk - (h / 6.0) * (fk + 4.0 * fc + fk1);

        const int row = k * m_state_len;
        constraint_values(Eigen::seqN(row, m_state_len)) = c_def;
    }

    return constraint_values;
}

void SimpsonDefectConstraints::FillJacobianBlock(
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

int SimpsonDefectConstraints::getVarTypeLen(const VariableType var_type) const
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

void SimpsonDefectConstraints::FillJacobianWrt(
    const VariableType var_type,
    ifopt::Component::Jacobian &jac_block) const
{
    const Eigen::VectorXd state_vars = m_state_vars->GetValues();
    const Eigen::VectorXd control_vars = m_ctrl_vars->GetValues();

    const Eigen::VectorXd state_mid_vars = m_state_mid_vars->GetValues();
    const Eigen::VectorXd control_mid_vars = m_ctrl_mid_vars->GetValues();

    const double h
        = m_dt_segment;  // will probably just rename m_dt_segment later

    // use list of triplets to simplify and avoid costly random
    // insertions when constructing the final sparse jacobian matrix.
    std::vector<Eigen::Triplet<double>> triplet_list;
    const int var_type_len = getVarTypeLen(var_type);
    const int num_nonzero_submatrices = 2;
    const int num_defect_vec_eqns = m_num_segments;
    triplet_list.reserve(m_state_len * var_type_len * num_nonzero_submatrices
                         * num_defect_vec_eqns);

    // k indexes a trajectory segment,
    // j indexes the variable block whose contribution is being inserted into
    // the full jacobian for segment k.
    //
    // for knot variable sets Simpson defect for segment k depends on both
    // endpoint knot blocks so use j = k and j = k + 1.
    //
    // for midpoint variable sets there is only one
    // variable block associated with segment k so only use j = k.
    //
    // j_max is k + 2 for knot variables and k + 1 for midpoint
    // variables.
    const size_t k_max = static_cast<size_t>(m_num_segments);

    for (size_t k{}; k < k_max; ++k) {
        const bool is_mid = (var_type == VariableType::STATE_MID
                             || var_type == VariableType::CONTROL_MID);

        // midpoint variable sets have one block per segment so only j = k is
        // vald.
        // knot variable sets have two neighboring blocks for
        // segment k, j = k an j = k + 1.
        const size_t j_max = is_mid ? (k + 1) : (k + 2);

        for (size_t j = k; j < j_max; ++j) {
            // j is used here instead of k because the loop bounds above
            // guarantee that j already refers to the correct variable block
            //
            // midpoint  j = k is usd.
            //
            // knot j = k and j = k + 1 are used.
            auto statej
                = is_mid
                      ? state_mid_vars(
                            Eigen::seqN(j * m_state_len, m_state_len))
                      : state_vars(Eigen::seqN(j * m_state_len, m_state_len));
            auto controlj
                = is_mid ? control_mid_vars(
                               Eigen::seqN(j * m_control_len, m_control_len))
                         : control_vars(
                               Eigen::seqN(j * m_control_len, m_control_len));

            const double tj = is_mid ? ((static_cast<double>(j) + 0.5) * h)
                                     : (m_dt_segment * j);

            // segment k has rows starting at k * m_state_len
            const int row_start = static_cast<int>(k) * m_state_len;

            // variable block j has columns starting at j * var_type_len
            const int col_start = static_cast<int>(j) * var_type_len;

            ifopt::Component::Jacobian jac_constraints_wrt_var
                = jacConstraintsWrtVar(var_type, k, j, statej, controlj, tj);

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

ifopt::Component::Jacobian SimpsonDefectConstraints::jacConstraintsWrtVar(
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
            if (j != k) {
                ifopt::Component::Jacobian Z(m_state_len, m_state_len);
                Z.setZero();
                return Z;
            }

            ifopt::Component::Jacobian Ac
                = m_jac_dyn_wrt_state_fn(state, control, time);
            ifopt::Component::Jacobian dcd_dxc = -(2.0 * h / 3.0) * Ac;
            return dcd_dxc;
        }

        case VariableType::CONTROL_MID: {
            if (j != k) {
                ifopt::Component::Jacobian Z(m_state_len, m_control_len);
                Z.setZero();
                return Z;
            }

            ifopt::Component::Jacobian Bc
                = m_jac_dyn_wrt_control_fn(state, control, time);
            ifopt::Component::Jacobian dcd_duc = -(2.0 * h / 3.0) * Bc;
            return dcd_duc;
        }
    }

    assert(false);
    ifopt::Component::Jacobian Z(m_state_len, m_state_len);
    Z.setZero();
    return Z;
}

ifopt::Component::Jacobian SimpsonDefectConstraints::jacConstraintsWrtState(
    const size_t k,
    const size_t j,
    const Eigen::VectorXd &state,
    const Eigen::VectorXd &control,
    const double time) const
{
    const double h = m_dt_segment;

    if (!(j == k || j == k + 1)) {
        ifopt::Component::Jacobian Z(m_state_len, m_state_len);
        Z.setZero();
        return Z;
    }

    const bool left = (j == k);
    ifopt::Component::Jacobian dfj_dxj
        = m_jac_dyn_wrt_state_fn(state, control, time);

    ifopt::Component::Jacobian I(m_state_len, m_state_len);
    I.setIdentity();

    const double s_def_I = left ? -1.0 : +1.0;

    ifopt::Component::Jacobian dcd_dx = ( s_def_I )*I + (-(h / 6.0)) * dfj_dxj;

    return dcd_dx;
}

ifopt::Component::Jacobian SimpsonDefectConstraints::jacConstraintsWrtControl(
    const size_t k,
    const size_t j,
    const Eigen::VectorXd &state,
    const Eigen::VectorXd &control,
    const double time) const
{
    const double h = m_dt_segment;

    if (!(j == k || j == k + 1)) {
        ifopt::Component::Jacobian Z(m_state_len, m_control_len);
        Z.setZero();
        return Z;
    }

    ifopt::Component::Jacobian B
        = m_jac_dyn_wrt_control_fn(state, control, time);

    ifopt::Component::Jacobian dcd_du = (-(h / 6.0)) * B;
    return dcd_du;
}
