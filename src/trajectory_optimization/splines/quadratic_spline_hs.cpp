#include "quadratic_spline_hs.hpp"

#include <exception>
#include <iostream>
#include <polynomial_interpolation.hpp>

QuadraticSpline::QuadraticSpline(std::vector<Eigen::VectorXd> func_vals,
                                 const double start_time,
                                 const double duration)
    : m_func_vals{std::move(func_vals)}
    , m_start_time{start_time}
    , m_duration{duration}
{
    if (m_func_vals.size() < 3) {
        throw std::invalid_argument(
            "QuadraticSpline. Need at least 3 values for one HS interval.");
    }

    if (((m_func_vals.size() - 1) % 2) != 0) {
        throw std::invalid_argument(
            "QuadraticSpline. HS data must contain 2*N + 1 values.");
    }

    if (m_duration <= 0.0) {
        throw std::invalid_argument(
            "QuadraticSpline. duration must be positive.");
    }

    const Eigen::Index dim = m_func_vals.front().size();
    for (const auto& v : m_func_vals) {
        if (v.size() != dim) {
            throw std::invalid_argument(
                "QuadraticSpline. all vectors must have the same dimension.");
        }
    }
}

Eigen::VectorXd QuadraticSpline::getValue(const double time) const
{
    const double end_time = m_start_time + m_duration;
    if ((time < m_start_time) || (time > end_time)) {
        std::ostringstream os;
        os << "QuadraticSpline. time out of bounds. time: " << time
           << ", start time: " << m_start_time
           << ", end time: " << end_time;
        throw std::invalid_argument(os.str());
    }

    const int num_segments = static_cast<int>((m_func_vals.size() - 1) / 2);
    const double alpha = (time - m_start_time) / m_duration;
    const int i_seg = static_cast<int>(alpha * num_segments);

    if (i_seg == num_segments) {
        return m_func_vals.back();
    }

    const double h = m_duration / static_cast<double>(num_segments);
    const double t0 = m_start_time + static_cast<double>(i_seg) * h;
    const double tau = (time - t0) / h;   // tau in [0, 1]

    const Eigen::VectorXd& x0 = m_func_vals[2 * i_seg];
    const Eigen::VectorXd& xc = m_func_vals[2 * i_seg + 1];
    const Eigen::VectorXd& x1 = m_func_vals[2 * i_seg + 2];

    const double L0 = 2.0 * (tau - 0.5) * (tau - 1.0);
    const double Lc = -4.0 * tau * (tau - 1.0);
    const double L1 = 2.0 * tau * (tau - 0.5);

    return L0 * x0 + Lc * xc + L1 * x1;
}
