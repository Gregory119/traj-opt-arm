#pragma once

#include <Eigen/Core>

#include <vector>

class CubicSpline
{
public:
    /*
     * Piecewise cubic Hermite spline with uniformly spaced knot times.
     *
     * @param func_vals Function values at knot points.
     * @param grad_vals Time-derivative values at knot points.
     * @param start_time Start time of the full spline.
     * @param duration Total duration of the full spline.
     */
    CubicSpline(std::vector<Eigen::VectorXd> func_vals,
                std::vector<Eigen::VectorXd> grad_vals,
                double start_time,
                double duration);


    // Get the value of the spline at a particular time.
    Eigen::VectorXd getValue(double time) const;

private:
    std::vector<Eigen::VectorXd> m_func_vals;
    std::vector<Eigen::VectorXd> m_grad_vals;
    double m_start_time;
    double m_duration;
};