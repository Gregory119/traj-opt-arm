#include "control_effort_HS_cost.hpp"

#include <cassert>

ControlEffortHermSimpCost::ControlEffortHermSimpCost(
    const std::string &cost_name,
    const std::string &ctrl_vars,
    const int ctrl_len,
    const double dt_segment)
    : CostTerm(cost_name)
    , m_ctrl_vars{ctrl_vars}
    , m_ctrl_len{ctrl_len}
    , m_dt_segment{dt_segment}
{}

double ControlEffortHermSimpCost::GetCost() const
{
    const Eigen::VectorXd ctrl_vars
        = GetVariables()->GetComponent(m_ctrl_vars)->GetValues();
    assert(ctrl_vars.size() % m_ctrl_len == 0);
    const int num_vectors = ctrl_vars.size() / m_ctrl_len;

    // Integrate the control squared over the trajectory numerically using
    // trapezoidal quadrature. Loop through the control vectors in pairs
    // (u_k and u_(k+1)). The elements of each vector are squared and
    // summed.
    double cost{};

    for (int k{}; k < num_vectors - 1; ++k) {
        const auto uk = ctrl_vars(Eigen::seqN(k * m_ctrl_len, m_ctrl_len));
        const int k1 = k + 1;
        const auto uk1 = ctrl_vars(Eigen::seqN(k1 * m_ctrl_len, m_ctrl_len));

        cost += uk.squaredNorm() + uk1.squaredNorm();
    }
    cost = 0.5 * m_dt_segment * cost;

    return cost;
};

void ControlEffortHermSimpCost::FillJacobianBlock(
    std::string var_set,
    ifopt::Component::Jacobian &jac) const
{
    if (var_set == m_ctrl_vars) {
        // loop through each control vector
        const VectorXd ctrl_vars
            = GetVariables()->GetComponent(m_ctrl_vars)->GetValues();
        assert(ctrl_vars.size() % m_ctrl_len == 0);
        const int num_vectors = ctrl_vars.size() / m_ctrl_len;

        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(ctrl_vars.size());
        for (int k{}; k < num_vectors; ++k) {
            const auto uk = ctrl_vars(Eigen::seqN(k * m_ctrl_len, m_ctrl_len));
            for (int j{}; j < m_ctrl_len; ++j) {
                if ((k == 0) || (k == num_vectors - 1)) {
                    // jacobian w.r.t elements of either first or last
                    // control vector
                    triplets.push_back(
                        {0, k * m_ctrl_len + j, m_dt_segment * uk(j)});
                    continue;
                }
                // jacobian w.r.t elements of neither first or last control
                // vector
                triplets.push_back(
                    {0, k * m_ctrl_len + j, 2 * m_dt_segment * uk(j)});
            }
        }
        jac.setFromTriplets(triplets.cbegin(), triplets.cend());
    }
}
