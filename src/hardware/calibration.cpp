#include "calibration.hpp"

#include <cassert>
#include <exception>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <numbers>

using json = nlohmann::json;

// 12-bit encoder => 4096 tics per revolution, but the first tic is zero
static const int g_tic_p_rev = 4095;

Calibration::Calibration(const std::string &calibration_file_path)
{
    // open file
    std::ifstream f(calibration_file_path);
    if (!f.is_open()) {
        std::ostringstream os;
        os << "Calibration() - failed to open calibration file: "
           << calibration_file_path;
        throw std::invalid_argument(os.str());
    }

    std::map<int, ServoPosRange> tmp = {{1, ServoPosRange{751, 3470}},
                                        {2, ServoPosRange{920, 3281}},
                                        {3, ServoPosRange{933, 3137}},
                                        {4, ServoPosRange{875, 3215}},
                                        {5, ServoPosRange{221, 4022}},
                                        {6, ServoPosRange{2037, 3499}}};

    // extract data
    json data = json::parse(f);
    for (auto &[_, value] : data.items()) {
        const int sid = value["id"].get<int>();
        m_pos_tic_ranges[sid] = {.pos_min = value["range_min"].get<int>(),
                                 .pos_max = value["range_max"].get<int>()};
        std::cout << "range_min = " << value["range_min"].get<int>()
                  << ", range_max = " << value["range_max"].get<int>()
                  << std::endl;
        assert(m_pos_tic_ranges[sid].pos_min == tmp[sid].pos_min);
        assert(m_pos_tic_ranges[sid].pos_max == tmp[sid].pos_max);
    }
}

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

double Calibration::ticToPos(const int pos_tic,
                             const int sid,
                             const PosUnit unit) const
{
    double unit_p_rev{};
    switch (unit) {
        case PosUnit::RADIAN:
            unit_p_rev = 2 * std::numbers::pi;
            break;
        case PosUnit::DEGREE:
            unit_p_rev = 360.0;
            break;
    }
    const double unit_p_tic = unit_p_rev / static_cast<double>(g_tic_p_rev);
    const double pos_unit = (pos_tic - getZeroTic(sid)) * unit_p_tic;
    return pos_unit;
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
