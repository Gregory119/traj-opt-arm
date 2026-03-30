#include "quadratic_spline.hpp"

#include <exception>
#include <iostream>
#include <polynomial_interpolation.hpp>

QuadraticSpline::QuadraticSpline(std::vector<Eigen::VectorXd> func_vals,
                                 std::vector<Eigen::VectorXd> grad_vals,
                                 const double start_time,
                                 const double duration)
    : m_func_vals{std::move(func_vals)}
    , m_grad_vals{std::move(grad_vals)}
    , m_start_time{start_time}
    , m_duration{duration}
{
    if (m_func_vals.empty()) {
        throw std::invalid_argument("QuadraticSpline. empty values.");
    }
    //assert(func_vals.size() == grad_vals.size());
    if (m_func_vals.size() < 2) {
        throw std::invalid_argument("QuadraticSpline. Need at least 2 knot values.");
    }

    if (m_grad_vals.size() != m_func_vals.size() - 1) {
        throw std::invalid_argument(
            "QuadraticSpline. midpoint/value count must be exactly one less than knot count.");
    }

    const Eigen::Index dim = m_func_vals.front().size();
    if (dim == 0) {
        throw std::invalid_argument("QuadraticSpline. knot vectors must be non-empty.");
    }

    for (size_t i = 0; i < m_func_vals.size(); ++i) {
        if (m_func_vals[i].size() != dim) {
            throw std::invalid_argument(
                "QuadraticSpline. all knot vectors must have the same size.");
        }
    }

    for (size_t i = 0; i < m_grad_vals.size(); ++i) {
        if (m_grad_vals[i].size() != dim) {
            throw std::invalid_argument(
                "QuadraticSpline. all midpoint vectors must have the same size as the knot vectors.");
        }
    }

}

Eigen::VectorXd QuadraticSpline::getValue(const double time) const
{
    // check time bounds
    const double end_time = m_start_time + m_duration;
    if ((time < m_start_time) || (time > end_time)) {
        std::ostringstream os;
        os << "QuadraticSpline. time out of bounds. time: " << time
           << ", start time: " << m_start_time << ", end time: " << end_time;
        throw std::invalid_argument(os.str());
    }

    // Get the index to the start time of the segment.
    const int num_segments = m_func_vals.size() - 1;
    const double alpha = (time - m_start_time) / m_duration;
    const int i_start = static_cast<int>(alpha * num_segments);

    // if at end, then return last value
    if (i_start == num_segments) {
        return m_func_vals[i_start];
    }

    // get start time
    const double beta = static_cast<double>(i_start) / num_segments;
    const double ti = beta * m_duration + m_start_time;
    // duration of time segment
    const double dt_max = m_duration / num_segments;
    // time relative to start time of segment
    const double dt = time - ti;
    // std::cout << "QuadraticSpline(). time=" << time << ", alpha=" << alpha
    //           << ", num_segments=" << num_segments << ", i_start=" << i_start
    //           << ", ti=" << ti << ", beta=" << beta << ", dt=" << dt
    //           << ", dt_max=" << dt_max << std::endl;

    /*
    // quadratic polynomial interpolation
    return interp_quad(m_func_vals[i_start],      // xi
                       m_grad_vals[i_start],      // dxi
                       m_grad_vals[i_start + 1],  // dxf
                       dt_max,
                       dt);
    */
    return interp_quad_midpoint(m_func_vals[i_start],      // x_i
                            m_grad_vals[i_start],      // x_c (midpoint value)
                            m_func_vals[i_start + 1],  // x_{i+1}
                            dt_max,
                            dt);
}
