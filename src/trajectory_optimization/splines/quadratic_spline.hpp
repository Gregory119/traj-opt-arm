#pragma once

#include <Eigen/Dense>
#include <map>
#include <vector>

// A quadratic spline is a piece-wise function with quadratic polynomials for
// each time segment.
class QuadraticSpline
{
public:
    enum class ConstraintType {
        Gradient,
        Midpoint
    };

    /*
     * @param func_vals Function values at knot points
     * @param constraint_vals Values interpreted according to constraint_type:
     *        - Gradient: time derivative values at knot points
     *        - Midpoint: function values at segment midpoints
     */
    QuadraticSpline(std::vector<Eigen::VectorXd> func_vals,
                    std::vector<Eigen::VectorXd> constraint_vals,
                    const ConstraintType constraint_type,
                    const double start_time,
                    const double duration);

    // Get the value of the spline at a particular time.
    Eigen::VectorXd getValue(const double time) const;

private:
    const std::vector<Eigen::VectorXd> m_func_vals;
    const std::vector<Eigen::VectorXd> m_constraint_vals;
    const ConstraintType m_constraint_type;

    const double m_start_time;
    const double m_duration;
};
