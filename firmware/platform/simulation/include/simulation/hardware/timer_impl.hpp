#pragma once

#include <chrono>
#include <micromouse/hardware/timer.hpp>

class TimerImpl : public hardware::Timer {
  std::chrono::high_resolution_clock::time_point m_start_time_point;
  std::chrono::high_resolution_clock::time_point m_stop_time_point;

  bool m_is_running = false;

 public:
  TimerImpl();

  void reset() override;
  void start() override;
  void stop() override;

  uint32_t elapsed_ms() const override;
};
