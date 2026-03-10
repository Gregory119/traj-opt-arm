#pragma once

#include <Eigen/Dense>
#include <map>
#include <vector>

// A quadratic spline is a piece-wise function with quadratic polynomials for
// each time segment.
class QuadraticSpline
{
public:
    /*
     * @param func_vals Function values at knot points.
     * @parma grad_vals Values of the time derivative of the function to be
     * represented at the knot points
     */
    QuadraticSpline(std::vector<Eigen::VectorXd> func_vals,
                    std::vector<Eigen::VectorXd> grad_vals,
                    const double start_time,
                    const double duration);

    // Get the value of the spline at a particular time.
    Eigen::VectorXd getValue(const double time) const;

private:
    const std::vector<Eigen::VectorXd> m_func_vals;
    const std::vector<Eigen::VectorXd> m_grad_vals;

    const double m_start_time;
    const double m_duration;
};
