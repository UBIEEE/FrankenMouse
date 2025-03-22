#include <simulation/hardware/timer_impl.hpp>

using namespace hardware;

TimerImpl::TimerImpl() {
  reset();
}

void TimerImpl::reset() {
  m_start_time_point = m_stop_time_point =
      std::chrono::high_resolution_clock::now();

  m_is_running = false;
}

void TimerImpl::start() {
  if (m_is_running)
    return;

  m_start_time_point = std::chrono::high_resolution_clock::now();
  m_is_running = true;
}

void TimerImpl::stop() {
  if (!m_is_running)
    return;

  m_stop_time_point = std::chrono::high_resolution_clock::now();
  m_is_running = false;
}

uint32_t TimerImpl::elapsed_ms() const {
  auto end_time_point = m_is_running ? std::chrono::high_resolution_clock::now()
                                     : m_stop_time_point;

  return std::chrono::duration_cast<std::chrono::milliseconds>(
             end_time_point - m_start_time_point)
      .count();
}

std::unique_ptr<Timer> make_platform_timer() {
  return std::make_unique<TimerImpl>();
}
