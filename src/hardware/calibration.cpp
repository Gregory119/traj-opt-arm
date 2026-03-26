#include "calibration.hpp"

#include <numbers>

static const int g_tic_p_rev = 4096;

Calibration::Calibration(const std::vector<ServoPosRange> &pos_tic_ranges)
    : m_pos_tic_ranges{pos_tic_ranges}
{}

bool Calibration::inRangeTic(const int pos_tic, const int sid) const
{
    return (pos_tic > m_pos_tic_ranges[sid].pos_min)
           && (pos_tic < m_pos_tic_ranges[sid].pos_max);
}

bool Calibration::inRangePos(const double pos,
                             const int sid,
                             const PosUnit unit) const
{
    // keep position in specified units for convenient debug logs by converting
    // tic range to unit range
    double unit_p_tic{};
    switch (unit) {
        case PosUnit::RADIAN:
            unit_p_tic = 2 * std::numbers::pi / g_tic_p_rev;
            break;

        case PosUnit::DEGREE:
            unit_p_tic = 360.0 / g_tic_p_rev;
            break;
    }
    const double pos_max_unit
        = (m_pos_tic_ranges[sid].pos_max - getZeroTic(sid)) * unit_p_tic;
    const double pos_min_unit
        = (m_pos_tic_ranges[sid].pos_min - getZeroTic(sid)) * unit_p_tic;
    return (pos > pos_min_unit) && (pos < pos_max_unit);
}

int Calibration::posToTic(const double pos,
                          const int sid,
                          const PosUnit unit) const
{
    // unit => output position unit
    double rev_p_unit{};
    switch (unit) {
        case PosUnit::RADIAN:
            rev_p_unit = 0.5 / std::numbers::pi;
            break;
        case PosUnit::DEGREE:
            rev_p_unit = 1.0 / 360.0;
            break;
    }
    const double tic_p_unit = static_cast<double>(g_tic_p_rev) * rev_p_unit;
    const double pos_tic = pos * tic_p_unit + getZeroTic(sid);
    return pos_tic;
}

int Calibration::getZeroTic(const int sid) const
{
    return (m_pos_tic_ranges[sid].pos_max + m_pos_tic_ranges[sid].pos_min) / 2;
}
