#include "hardware/timer_impl.hpp"

#include "stm32wbxx_hal.h"

TimerImpl::TimerImpl() {
  reset();
}

void TimerImpl::reset() {
  m_time_start_ms = m_time_stop_ms = HAL_GetTick();
}

void TimerImpl::start() {
  if (m_is_running)
    return;

  m_time_start_ms = HAL_GetTick();
  m_is_running = true;
}

void TimerImpl::stop() {
  if (!m_is_running)
    return;

  m_time_stop_ms = HAL_GetTick();
  m_is_running = false;
}

uint32_t TimerImpl::elapsed_ms() const {
  uint32_t end_time_ms = m_is_running ? HAL_GetTick() : m_time_stop_ms;

  return end_time_ms - m_time_start_ms;
}

std::unique_ptr<hardware::Timer> make_platform_timer() {
  return std::make_unique<TimerImpl>();
}
