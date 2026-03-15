#include "control_effort_HS_cost.hpp"

#include <cassert>

ControlEffortHermSimpCost::ControlEffortHermSimpCost(
    const std::string &cost_name,
    const std::string &ctrl_vars_name,
    const std::string &ctrl_vars_mid_name,
    const int ctrl_len,
    const double dt_segment)
    : CostTerm(cost_name)
    , m_ctrl_vars_name{ctrl_vars_name}
    , m_ctrl_vars_mid_name{ctrl_vars_mid_name}
    , m_ctrl_len{ctrl_len}
    , m_dt_segment{dt_segment}
{}

double ControlEffortHermSimpCost::GetCost() const
{
    const Eigen::VectorXd ctrl_vars
        = GetVariables()->GetComponent(m_ctrl_vars_name)->GetValues();
    const Eigen::VectorXd ctrl_mid_vars
        = GetVariables()->GetComponent(m_ctrl_vars_mid_name)->GetValues();
    assert(ctrl_vars.size() % m_ctrl_len == 0);
    assert(ctrl_mid_vars.size() % m_ctrl_len == 0);
    const int num_knots = ctrl_vars.size() / m_ctrl_len;
    const int num_segments = ctrl_mid_vars.size() / m_ctrl_len;

    assert(num_knots == num_segments + 1);

    // Integrate the control squared over the trajectory numerically using
    // trapezoidal quadrature. Loop through the control vectors in pairs
    // (u_k and u_(k+1)). The elements of each vector are squared and
    // summed.
    double cost{0.0};

    for (int k = 0; k < num_segments; ++k) {
        const auto uk  = ctrl_vars(Eigen::seqN(k * m_ctrl_len, m_ctrl_len));
        const auto uc  = ctrl_mid_vars(Eigen::seqN(k * m_ctrl_len, m_ctrl_len));
        const auto uk1 = ctrl_vars(Eigen::seqN((k + 1) * m_ctrl_len, m_ctrl_len));

        cost += uk.squaredNorm() + 4.0 * uc.squaredNorm() + uk1.squaredNorm();
    }

    cost *= (m_dt_segment / 6.0);
    return cost;
};

void ControlEffortHermSimpCost::FillJacobianBlock(
    std::string var_set,
    ifopt::Component::Jacobian &jac) const
{
    if (var_set == m_ctrl_vars_name) {
        const VectorXd ctrl_vars
            = GetVariables()->GetComponent(m_ctrl_vars_name)->GetValues();
        assert(ctrl_vars.size() % m_ctrl_len == 0);

        const int num_knots = ctrl_vars.size() / m_ctrl_len;

        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(ctrl_vars.size());

        for (int k = 0; k < num_knots; ++k) {
            const auto uk = ctrl_vars(Eigen::seqN(k * m_ctrl_len, m_ctrl_len));

            const double coeff =
                (k == 0 || k == num_knots - 1)
                    ? (m_dt_segment / 3.0)
                    : (2.0 * m_dt_segment / 3.0);

            for (int j = 0; j < m_ctrl_len; ++j) {
                triplets.push_back({0, k * m_ctrl_len + j, coeff * uk(j)});
            }
        }

        jac.setFromTriplets(triplets.cbegin(), triplets.cend());
    }
    else if (var_set == m_ctrl_vars_mid_name) {
        const VectorXd ctrl_mid_vars
            = GetVariables()->GetComponent(m_ctrl_vars_mid_name)->GetValues();
        assert(ctrl_mid_vars.size() % m_ctrl_len == 0);

        const int num_mid = ctrl_mid_vars.size() / m_ctrl_len;

        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(ctrl_mid_vars.size());

        for (int k = 0; k < num_mid; ++k) {
            const auto uc = ctrl_mid_vars(Eigen::seqN(k * m_ctrl_len, m_ctrl_len));

            for (int j = 0; j < m_ctrl_len; ++j) {
                triplets.push_back(
                    {0, k * m_ctrl_len + j, (4.0 * m_dt_segment / 3.0) * uc(j)});
            }
        }

        jac.setFromTriplets(triplets.cbegin(), triplets.cend());
    }
}
