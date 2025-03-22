#include "hardware/encoder.hpp"

#include <micromouse/robot.h>
#include "main.h"

#include <cmath>
#include <numbers>

//
// Constants.
//

// Encoder constants.
static constexpr int8_t ENCODER_MAGNET_POLES = 6;
static constexpr float GEAR_RATIO = 20.f;
static constexpr float ENCODER_TICKS_PER_ROTATION =
    (ENCODER_MAGNET_POLES * GEAR_RATIO);

static constexpr float WHEEL_DIAMETER_MM = 25.f;
static constexpr float WHEEL_CIRCUMFERENCE_MM =
    (WHEEL_DIAMETER_MM * std::numbers::pi_v<float>);

static constexpr float ENCODER_TICKS_PER_MM =
    (ENCODER_TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE_MM);
static constexpr float ENCODER_MM_PER_TICK = (1.f / ENCODER_TICKS_PER_MM); // TODO: Measure instead of calculate

//
// Encoder functions.
//

Encoder::Data Encoder::update(uint16_t ticks) {
  const int32_t delta_ticks = calc_delta_ticks(ticks, m_last_ticks);

  m_last_ticks = ticks;

  m_ticks += delta_ticks;
  m_velocity = delta_ticks / ROBOT_UPDATE_PERIOD_S;

  return {.position_mm = m_ticks * ENCODER_MM_PER_TICK,
          .velocity_mmps = m_velocity * ENCODER_MM_PER_TICK};
}

int32_t Encoder::calc_delta_ticks(uint16_t current, uint16_t last) {
  int32_t delta_ticks = current - last;

  const bool signbit = std::signbit(delta_ticks);

  const int32_t abs_delta_ticks = signbit ? -delta_ticks : delta_ticks;

  // If the delta is big, there must have been an overflow.
  if (abs_delta_ticks > INT16_MAX) {
    // Add 0xFFFF when deep negative, subtract when positive.
    delta_ticks += UINT16_MAX * (signbit ? 1 : -1);
  }

  return delta_ticks;
}
