#pragma once

#include <ifopt/cost_term.h>

class HS071Cost final : public ifopt::CostTerm
{
public:
    HS071Cost()
        : CostTerm("cost")
    {}

    double GetCost() const override
    {
        const Eigen::VectorXd x = GetVariables()->GetComponent("var_set")->GetValues();
        return x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
    }
    
    void FillJacobianBlock(std::string var_set,
                           Jacobian &jac_block) const override
    {
        if (var_set == "var_set") {
            const Eigen::VectorXd x
                = GetVariables()->GetComponent("var_set")->GetValues();

            // At this point the nonzero elements of the jacobian have to be set
            // (they don't exist yet). Fill in along the major order - ifopt
            // uses row major order for the jacobian sparse matrix by default so
            // fill row by row. This could be optimized by reserving memory for
            // each major axis (see
            // https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialSparse.html).

            // note that this is the jacobian is the transpose of the gradient
            jac_block.insert(0, 0) = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
            jac_block.insert(0, 1) = x[0] * x[3];
            jac_block.insert(0, 2) = x[1] * x[3] + 1;
            jac_block.insert(0, 3) = x[0] * (x[0] + x[1] + x[2]);
        }
    }

private:
    Eigen::Vector4d m_x;
};
