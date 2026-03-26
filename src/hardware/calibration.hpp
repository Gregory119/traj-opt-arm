#pragma once

#include <map>

// servo position unit ranges
struct ServoPosRange
{
    int pos_min;
    int pos_max;
};

enum class PosUnit
{
    RADIAN,
    DEGREE
};

class Calibration
{
public:
    // todo: change this vector to a map of sid to range
    Calibration(const std::map<int, ServoPosRange> &pos_tic_ranges);

    bool inRangeTic(const int pos_tic, const int sid) const;

    /*
     * @param unit Unit of the input position.
     */
    bool inRangePos(const double pos,
                    const int sid,
                    const PosUnit unit = PosUnit::RADIAN) const;

    /*
     * @param unit Unit of the input position.
     */
    int posToTic(const double pos,
                 const int sid,
                 const PosUnit unit = PosUnit::RADIAN) const;

private:
    // get the zero radian or degree position in tics
    int getZeroTic(const int sid) const;

    const std::map<int, ServoPosRange> &m_pos_tic_ranges;
};
