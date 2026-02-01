#pragma once

#include <functional>
#include <optional>

class PeriodicSimTimer
{
public:
    PeriodicSimTimer(const double period, const std::function<void()> &cb);

    void update(const double sim_time);

    void reset();

private:
    const double m_period;
    std::optional<double> m_last_trigger_time;
    const std::function<void()> m_cb;
};
