#include "quadratic_spline.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <polynomial_interpolation.hpp>

QuadraticSpline::QuadraticSpline(std::vector<Eigen::VectorXd> func_vals,
                                 std::vector<Eigen::VectorXd> constraint_vals,
                                 const ConstraintType constraint_type,
                                 const double start_time,
                                 const double duration)
    : m_func_vals{std::move(func_vals)}
    , m_constraint_vals{std::move(constraint_vals)}
    , m_constraint_type{constraint_type}
    , m_start_time{start_time}
    , m_duration{duration}
{
    if (m_func_vals.empty()) {
        throw std::invalid_argument("QuadraticSpline. empty values.");
    }

    if (m_duration <= 0.0) {
        throw std::invalid_argument(
            "QuadraticSpline. duration must be positive.");
    }
    // size consistency is checked below based on m_constraint_type
    if (m_func_vals.size() < 2) {
        throw std::invalid_argument("QuadraticSpline. Need at least 2 knot values.");
    }

    switch (m_constraint_type) {
    case ConstraintType::Gradient:
        if (m_constraint_vals.size() != m_func_vals.size()) {
            throw std::invalid_argument(
                "QuadraticSpline. gradient count must equal knot count.");
        }
        break;

    case ConstraintType::Midpoint:
        if (m_constraint_vals.size() != m_func_vals.size() - 1) {
            throw std::invalid_argument(
                "QuadraticSpline. midpoint/value count must be exactly one less than knot count.");
        }
        break;

    default:
        throw std::invalid_argument("QuadraticSpline. invalid constraint type.");
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

    for (size_t i = 0; i < m_constraint_vals.size(); ++i) {
        if (m_constraint_vals[i].size() != dim) {
            throw std::invalid_argument(
                "QuadraticSpline. all constraint vectors must have the same size as the knot vectors.");
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

    switch (m_constraint_type) {
    case ConstraintType::Gradient:
        return interp_quad(m_func_vals[i_start],          // x_i
                           m_constraint_vals[i_start],    // dx_i
                           m_constraint_vals[i_start + 1],// dx_{i+1}
                           dt_max,
                           dt);

    case ConstraintType::Midpoint:
        return interp_quad_midpoint(m_func_vals[i_start],       // x_i
                                    m_constraint_vals[i_start], // x_c
                                    m_func_vals[i_start + 1],   // x_{i+1}
                                    dt_max,
                                    dt);

    default:
        throw std::invalid_argument("QuadraticSpline. invalid constraint type.");
    }
}
