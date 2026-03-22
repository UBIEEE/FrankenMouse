#include <simulation/hardware/timer_impl.hpp>

using namespace hardware;

TimerImpl::TimerImpl() {
  reset();
}

void TimerImpl::reset() {
  m_start_time_point = m_stop_time_point = std::chrono::high_resolution_clock::now();
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

units::millisecond_t TimerImpl::get() const {
  auto end_time_point = m_is_running ? std::chrono::high_resolution_clock::now() : m_stop_time_point;
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_point - m_start_time_point);

  return units::millisecond_t{static_cast<float>(duration.count())};
}

const auto start_time_point = std::chrono::high_resolution_clock::now();

units::millisecond_t get_system_timestamp() {
  auto now = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_point);

  return units::millisecond_t{static_cast<float>(duration.count())};
}

std::unique_ptr<Timer> make_platform_timer() {
  return std::make_unique<TimerImpl>();
}
