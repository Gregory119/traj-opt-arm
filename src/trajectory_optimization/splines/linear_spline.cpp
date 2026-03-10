#include "linear_spline.hpp"

#include <exception>
#include <polynomial_interpolation.hpp>

LinearSpline::LinearSpline(std::vector<Eigen::VectorXd> func_vals,
                           const double start_time,
                           const double duration)
    : m_func_vals{std::move(func_vals)}
    , m_start_time{start_time}
    , m_duration{duration}
{
    if (m_func_vals.empty()) {
        throw std::invalid_argument("LinearSpline. empty values.");
    }
}

Eigen::VectorXd LinearSpline::getValue(const double time) const
{
    // check time bounds
    const double end_time = m_start_time + m_duration;
    if ((time < m_start_time) || (time > end_time)) {
        std::ostringstream os;
        os << "LinearSpline. time out of bounds. time: " << time
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
    // std::cout << "LinearSpline(). time=" << time << ", alpha=" << alpha
    //           << ", num_segments=" << num_segments << ", i_start=" << i_start
    //           << ", ti=" << ti << ", beta=" << beta << ", dt=" << dt
    //           << ", dt_max=" << dt_max << std::endl;

    // linear polynomial interpolation
    return interp_linear(m_func_vals[i_start],      // xi
                         m_func_vals[i_start + 1],  // xf
                         dt_max,
                         dt);
}
