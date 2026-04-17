#include "cubic_spline.hpp"

#include <cassert>
#include <sstream>
#include <stdexcept>
#include <utility>

CubicSpline::CubicSpline(std::vector<Eigen::VectorXd> func_vals,
                         std::vector<Eigen::VectorXd> grad_vals,
                         const double start_time,
                         const double duration)
    : m_func_vals{std::move(func_vals)}
    , m_grad_vals{std::move(grad_vals)}
    , m_start_time{start_time}
    , m_duration{duration}
{
    if (m_func_vals.size() < 2) {
        throw std::invalid_argument(
            "CubicSpline. Need at least 2 knot values.");
    }

    if (m_func_vals.size() != m_grad_vals.size()) {
        throw std::invalid_argument(
            "CubicSpline. func_vals and grad_vals must have the same size.");
    }

    if (m_duration <= 0.0) {
        throw std::invalid_argument(
            "CubicSpline. duration must be positive.");
    }

    const Eigen::Index dim = m_func_vals.front().size();
    if (dim == 0) {
        throw std::invalid_argument(
            "CubicSpline. knot vectors must be non-empty.");
    }

    for (size_t i = 0; i < m_func_vals.size(); ++i) {
        if (m_func_vals[i].size() != dim) {
            throw std::invalid_argument(
                "CubicSpline. all function vectors must have the same size.");
        }
        if (m_grad_vals[i].size() != dim) {
            throw std::invalid_argument(
                "CubicSpline. all gradient vectors must have the same size as "
                "the function vectors.");
        }
    }
}

Eigen::VectorXd CubicSpline::getValue(const double time) const
{    /*
     * Reference: Kelly, "An Introduction to Trajectory Optimization:
     * How to Do Your Own Direct Collocation".
     */
    const double end_time = m_start_time + m_duration;
    if ((time < m_start_time) || (time > end_time)) {
        std::ostringstream os;
        os << "CubicSpline. time out of bounds. time: " << time
           << ", start time: " << m_start_time
           << ", end time: " << end_time;
        throw std::invalid_argument(os.str());
    }
    // Get the index to the start time of the segment
    const int num_segments = static_cast<int>(m_func_vals.size()) - 1;
    const double alpha = (time - m_start_time) / m_duration;
    const int i_start = static_cast<int>(alpha * num_segments);

    if (i_start == num_segments) {
        return m_func_vals[i_start];
    }

    const double dt_max = m_duration / static_cast<double>(num_segments);
    const double ti = m_start_time + static_cast<double>(i_start) * dt_max;
    const double dt = time - ti;
    const double s = dt / dt_max;

    // Cubic Hermite basis functions on s in [0, 1]
    const double h00 = 2.0 * s * s * s - 3.0 * s * s + 1.0;
    const double h10 = s * s * s - 2.0 * s * s + s;
    const double h01 = -2.0 * s * s * s + 3.0 * s * s;
    const double h11 = s * s * s - s * s;

    const Eigen::VectorXd& x0 = m_func_vals[i_start];
    const Eigen::VectorXd& dx0 = m_grad_vals[i_start];
    const Eigen::VectorXd& x1 = m_func_vals[i_start + 1];
    const Eigen::VectorXd& dx1 = m_grad_vals[i_start + 1];

    // dx0 and dx1 are time-derivatives, so scale tangent terms by dt_max.
    return h00 * x0
         + (h10 * dt_max) * dx0
         + h01 * x1
         + (h11 * dt_max) * dx1;
}