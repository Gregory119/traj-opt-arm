#pragma once

#include <ifopt/cost_term.h>

class ControlEffortHermSimpCost : public ifopt::CostTerm
{
public:
    ControlEffortHermSimpCost(const std::string &cost_name,
                              const std::string &ctrl_vars_name,
                              const std::string &ctrl_vars_mid_name,
                              const int ctrl_len,
                              const double dt_segment);

    double GetCost() const override;

    void FillJacobianBlock(std::string var_set,
                           ifopt::Component::Jacobian &jac) const override;

private:
    const std::string m_ctrl_vars_name;
    const std::string m_ctrl_vars_mid_name;
    const int m_ctrl_len;
    const double m_dt_segment;
};
