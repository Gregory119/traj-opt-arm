#pragma once

#include <ifopt/constraint_set.h>

class HS071Constraints final : public ifopt::ConstraintSet
{
public:
    HS071Constraints()
        : ConstraintSet(2, "constraint_set")
    {}

    Eigen::VectorXd GetValues() const override
    {
        const Eigen::VectorXd x
            = GetVariables()->GetComponent("var_set")->GetValues();
        Eigen::Vector2d g{
            x[0] * x[1] * x[2] * x[3],
            x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]};
        return g;
    }

    ifopt::Component::VecBound GetBounds() const override
    {
        ifopt::Component::VecBound bounds{{25, ifopt::inf}, {40.0, 40.0}};
        return bounds;
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
            jac_block.insert(0, 0) = x[1] * x[2] * x[3];
            jac_block.insert(0, 1) = x[0] * x[2] * x[3];
            jac_block.insert(0, 2) = x[0] * x[1] * x[3];
            jac_block.insert(0, 3) = x[0] * x[1] * x[2];
            jac_block.insert(1, 0) = 2 * x[0];
            jac_block.insert(1, 1) = 2 * x[1];
            jac_block.insert(1, 2) = 2 * x[2];
            jac_block.insert(1, 3) = 2 * x[3];
        }
    }

private:
    Eigen::Vector4d m_x;
};
