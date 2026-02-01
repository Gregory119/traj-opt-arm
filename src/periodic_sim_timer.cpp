#include "periodic_sim_timer.hpp"

PeriodicSimTimer::PeriodicSimTimer(const double period,
                                   const std::function<void()> &cb)
    : m_period{period}
    , m_cb{cb}
{}

void PeriodicSimTimer::update(const double sim_time)
{
    if (m_last_trigger_time) {
        const double dur = sim_time - *m_last_trigger_time;
        if (dur < m_period) {
            return;
        }
    }
    m_last_trigger_time = sim_time;
    m_cb();
}

void PeriodicSimTimer::reset()
{
    m_last_trigger_time.reset();
}
