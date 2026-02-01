#include "periodic_sim_timer.hpp"

PeriodicSimTimer::PeriodicSimTimer(
    const double period,
    const std::function<void(PeriodicSimTimer &)> &cb,
    const bool enable)
    : m_enable{enable}
    , m_period{period}
    , m_cb{cb}
{}

void PeriodicSimTimer::update(const double sim_time)
{
    if (!m_enable) {
        return;
    }
    // the timer must wait the duration before triggering (do not execute
    // callback on first update)
    if (!m_last_trigger_time) {
        m_last_trigger_time = sim_time;
        return;
    }
    const double dur = sim_time - *m_last_trigger_time;
    if (dur < m_period) {
        return;
    }
    // execute callback so set last trigger time
    m_last_trigger_time = sim_time;
    m_cb(*this);
}

void PeriodicSimTimer::reset(const std::optional<bool> &enable)
{
    m_last_trigger_time.reset();
    if (enable) {
        m_enable = *enable;
    }
}
