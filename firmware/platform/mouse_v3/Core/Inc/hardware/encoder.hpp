#pragma once

#include <cstdint>

class Encoder {
  uint16_t m_last_ticks = 0;

  int64_t m_ticks = 0;
  float m_velocity = 0.f;  // Ticks per second.

 public:
  struct Data {
    float position_mm = 0.f;
    float velocity_mmps = 0.f;
  };

 public:
  Data update(uint16_t ticks);

  void reset() {
    m_ticks = 0;
    m_velocity = 0.f;
  }

 private:
  int32_t calc_delta_ticks(uint16_t current, uint16_t last);
};
