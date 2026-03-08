#include "quadratic_spline.hpp"

#include <exception>
#include <quadratic_polynomial.hpp>

QuadraticSpline::QuadraticSpline(std::vector<Eigen::VectorXd> func_vals,
                                 std::vector<Eigen::VectorXd> grad_vals,
                                 const double start_time,
                                 const double duration)
    : m_func_vals{std::move(func_vals)}
    , m_grad_vals{std::move(grad_vals)}
    , m_start_time{start_time}
    , m_duration{duration}
{
    if (times.empty()) {
        throw std::invalid_argument("QuadraticSpline. empty times.");
    }
    assert(func_vals.size() == grad_vals.size());
    assert(func_vals.size() == times.size());
}

Eigen::VectorXd QuadraticSpline::getValue(const double time)
{
    // check time bounds
    const double end_time = m_start_time + m_duration;
    if ((time < m_start_time) || (time > end_time)) {
        std::ostringstream os;
        os << "QuadraticSpline. time out of bounds. time: " << time
           << ", start time: " << m_start_time << ", end time: " << m_end_time;
        throw std::invalid_argument(os.str());
    }

    // Get the index to the start time of the segment.
    const int num_segments = func_vals.size() - 1;
    const double alpha = (time - m_start_time) / m_duration;
    const int i_start = static_cast<int>(alpha * num_segments);

    // if at end, then return last value
    if (i_start == num_segments) {
        return func_vals(i_start);
    }

    // get start time
    const double ti = static_cast<int>(alpha) * m_duration + m_start_time;
    // duration of time segment
    const double dt_max = m_duration / num_segments;
    // time relative to start time of segment
    const double dt = time - ti;

    // quadratic polynomial interpolation
    return interp_quad(func_vals(i_start),      // xi
                       grad_vals(i_start),      // dxi
                       grad_vals(i_start + 1),  // dxf
                       dt_max,
                       dt);
}
