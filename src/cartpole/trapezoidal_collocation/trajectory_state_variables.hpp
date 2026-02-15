#pragma once

#include <cassert>

#include <ifopt/variable_set.h>

// Representation for the discrete state variables over the trajectory.
class TrajectoryStateVariables final : public ifopt::VariableSet
{
public:
    using BoundsFn = std::function<ifopt::Component::VecBound (const Eigen::VectorXd& x)>;
    
    /*
     * @param num_segments Number of time segments.
     * @param bounds Bounds of the discrete state variables.
     */
    TrajectoryStateVariables(const int num_segments,
                             const int state_len,
                             const BoundsFn& bounds_fn)
        : VariableSet(num_segments + 1, "traj_state"),
          m_bounds_fn{bounds_fn}
    {
        const int num_knot_points = num_segments + 1;
        const int num_state_vars = num_knot_points*state_len;
        assert(bounds.size() == num_state_vars);
        // initial guess for solution
        m_x = Eigen::VectorXd::Zeros(num_state_vars, 1);
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
        return m_bounds_fn(m_x);
    }

private:
    // This holds the discrete state values [x0, x1, ... , xN]. Note that each
    // state value holds as many elements as the dimension of the state, so for
    // a 2D state this will be [x00, x01, x10, x11, ... , xN0, xN1], which is
    // double the length.
    Eigen::VectorXd m_x;
    const std::function<void(const Eigen::VectorXd& x)> m_bounds_fn;
};
