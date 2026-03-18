#pragma once

#include <cassert>

#include <ifopt/variable_set.h>

// Representation for the discrete variables (eg. states or controls) over the
// trajectory.
class TrajectoryVariables final : public ifopt::VariableSet
{
public:
    /*
     * @param x_init Initial solution values (guessed solution).
     * @param bounds Bounds of the discrete state variables.
     */
    TrajectoryVariables(const std::string& name,
                        Eigen::VectorXd x_init,
                        ifopt::Component::VecBound bounds)
        : VariableSet(x_init.size(), name)
        , m_x{std::move(x_init)}
        , m_bounds{std::move(bounds)}
    {
        assert(m_bounds.size() == m_x.size());
    }

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
        return m_bounds;
    }

private:
    // This holds the discrete state or control values in a single vector. For n
    // state vectors each with k elements gives a single combined vector of n*k
    // elements. This is done by concatenating each of the state vectors in
    // order along a single axis. More specifically, for n state vectors x_0,
    // x_1, ..., x_i, ..., x_(n-1), where x_i=[x_i0, x_i1, ..., x_i(k-1)], this
    // combined vector becomes [x_00, x_01, ..., x_0(k-1), x_10, x_11, ...,
    // x_1(k-1), ..., x_(n-1)0, x_(n-1)1, ..., x_(n-1)(k-1)].
    Eigen::VectorXd m_x;
    ifopt::Component::VecBound m_bounds;
};
