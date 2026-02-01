#pragma once

#include <functional>
#include <optional>

class PeriodicSimTimer
{
public:
    PeriodicSimTimer(const double period,
                     const std::function<void(PeriodicSimTimer &)> &cb,
                     const bool enable = true);

    void update(const double sim_time);

    // if enable is not set then the enable state of the timer will remain
    // unchanged
    void reset(const std::optional<bool> &enable = std::nullopt);

private:
    bool m_enable = true;
    const double m_period;
    std::optional<double> m_last_trigger_time;
    const std::function<void(PeriodicSimTimer &)> m_cb;
};
