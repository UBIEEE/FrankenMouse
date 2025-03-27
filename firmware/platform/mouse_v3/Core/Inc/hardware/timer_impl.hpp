#pragma once

#include <chrono>
#include <cstdint>
#include <micromouse/hardware/timer.hpp>

class TimerImpl : public hardware::Timer {
  uint32_t m_time_start_ms = 0;
  uint32_t m_time_stop_ms = 0;

  bool m_is_running = false;

 public:
  TimerImpl();

  void reset() override;
  void start() override;
  void stop() override;

  uint32_t elapsed_ms() const override;
};
