#pragma once

#include <cassert>
#include <exception>
#include <sstream>

/*
 * Quadratic interpolation.
 *
 * @param xi Initial value.
 * @param dxi Initial gradient value (w.r.t time).
 * @param dxf Final gradient value (w.r.t time).
 * @param dt_max Time duration from start to end of polynomial.
 * @param dt Current time from start of polynomial.
 */
template <typename T>
T interp_quad(const T &xi,
              const T &dxi,
              const T &dxf,
              const double dt_max,
              const double dt)
{
    // check time bounds
    if ((dt < 0) || (dt > dt_max)) {
        std::ostringstream os;
        os << "interp_quad(). dt out of bounds. dt_max = " << dt_max
           << ", dt = " << dt;
        throw std::invalid_argument(os.str());
    }

    return xi + dxi * dt + dt * dt / 2 / dt_max * (dxf - dxi);
}

/*
 * Linear interpolation.
 *
 * @param xi Initial value.
 * @param xf Final value.
 * @param dt_max Time duration from start to end of polynomial.
 * @param dt Current time from start of polynomial.
 */
template <typename T>
T interp_linear(const T &xi, const T &xf, const double dt_max, const double dt)
{
    // check time bounds
    if ((dt < 0) || (dt > dt_max)) {
        std::ostringstream os;
        os << "interp_linear(). dt out of bounds. dt_max = " << dt_max
           << ", dt = " << dt;
        throw std::invalid_argument(os.str());
    }

    return xi + dt / dt_max * (xf - xi);
}


template <typename T>
T interp_quad_midpoint(const T &xi,
                       const T &xc,
                       const T &xf,
                       const double dt_max,
                       const double dt)
{
    if ((dt < 0) || (dt > dt_max)) {
        std::ostringstream os;
        os << "interp_quad_midpoint(). dt out of bounds. dt_max = " << dt_max
           << ", dt = " << dt;
        throw std::invalid_argument(os.str());
    }

    const double s = dt / dt_max;

    return (2.0 * s * s - 3.0 * s + 1.0) * xi
         + (4.0 * s - 4.0 * s * s) * xc
         + (2.0 * s * s - s) * xf;
}