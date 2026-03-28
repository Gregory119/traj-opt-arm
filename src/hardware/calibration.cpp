#include "calibration.hpp"

#include <cassert>
#include <iostream>
#include <numbers>

// 12-bit encoder => 4096 tics per revolution, but the first tic is zero
static const int g_tic_p_rev = 4095;

Calibration::Calibration(const std::map<int, ServoPosRange> &pos_tic_ranges)
    : m_pos_tic_ranges{pos_tic_ranges}
{}

bool Calibration::inRangeTic(const int pos_tic, const int sid) const
{
    if (!m_pos_tic_ranges.contains(sid)) {
        std::cout << "Calibration::inRangeTic() - sid does not exist in "
                     "position ranges. sid = "
                  << sid << std::endl;
        return false;
    }
    const auto range = m_pos_tic_ranges.find(sid)->second;
    return (pos_tic >= range.pos_min) && (pos_tic <= range.pos_max);
}

bool Calibration::inRangePos(const double pos,
                             const int sid,
                             const PosUnit unit) const
{
    if (!m_pos_tic_ranges.contains(sid)) {
        std::cout << "Calibration::inRangePos() - sid does not exist in "
                     "position ranges. sid = "
                  << sid << std::endl;
        return false;
    }

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
    const auto range = m_pos_tic_ranges.find(sid)->second;
    const double pos_max_unit = (range.pos_max - getZeroTic(sid)) * unit_p_tic;
    const double pos_min_unit = (range.pos_min - getZeroTic(sid)) * unit_p_tic;
    const bool res = (pos >= pos_min_unit) && (pos <= pos_max_unit);
    if (!res) {
        std::cout << "target position out of range. target pos = " << pos
                  << ", sid=" << sid << ", unit ";
        switch (unit) {
            case PosUnit::RADIAN:
                std::cout << "radian";
                break;

            case PosUnit::DEGREE:
                std::cout << "degree";
                break;
        }
        std::cout << ", max. pos [unit] = " << pos_max_unit
                  << ", min. pos [unit] = " << pos_min_unit
                  << ", max. pos [tic] = " << range.pos_max
                  << ", min. pos [tic] = " << range.pos_min
                  << ", zero [tic] = " << getZeroTic(sid) << std::endl;
    }
    return res;
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
    if (!m_pos_tic_ranges.contains(sid)) {
        std::cout << "Calibration::getZeroTic() - sid does not exist in "
                     "position ranges. sid = "
                  << sid << std::endl;
        return false;
    }
    const auto range = m_pos_tic_ranges.find(sid)->second;
    // end-effector is closed when zero
    if (sid == 6) {
        return range.pos_min;
    }
    return (range.pos_max + range.pos_min) / 2;
}
