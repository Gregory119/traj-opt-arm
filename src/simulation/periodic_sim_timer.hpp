#pragma once

#include <functional>
#include <optional>

class PeriodicSimTimer
{
public:
    using Fn
        = std::function<void(PeriodicSimTimer &timer, const double sim_time)>;

    PeriodicSimTimer(const double period,
                     const Fn &cb,
                     const bool enable = true);

    void update(const double sim_time);

    // if enable is not set then the enable state of the timer will remain
    // unchanged
    void reset(const std::optional<bool> &enable = std::nullopt);

    double getPeriod() const
    {
        return m_period;
    }

private:
    bool m_enable = true;
    const double m_period;
    std::optional<double> m_last_trigger_time;
    const Fn m_cb;
};
