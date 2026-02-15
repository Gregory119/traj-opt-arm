#pragma once

#include <ifopt/variable_set.h>

class HS071Variables final : public ifopt::VariableSet
{
public:
    HS071Variables()
        : VariableSet(4, "var_set")
        // initial guess for solution
        , m_x{1, 5, 5, 1}
    {}

    void SetVariables(const Eigen::VectorXd &x) override
    {
        m_x = x;
    }

    Eigen::VectorXd GetValues() const override
    {
        return m_x;
    }

    ifopt::Component::VecBound GetBounds() const override
    {
        ifopt::Component::VecBound bounds{{1, 5}, {1, 5}, {1, 5}, {1, 5}};
        return bounds;
    }

private:
    Eigen::Vector4d m_x;
};
