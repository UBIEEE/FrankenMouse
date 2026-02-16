#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <cstdint>

class Encoder {
  uint16_t m_last_ticks = 0;

  int64_t m_ticks = 0;
  float m_velocity = 0.f;  // Ticks per second.

 public:
  struct Data {
    units::millimeter_t position = 0_mm;
    units::millimeters_per_second_t velocity = 0_mmps;
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
