#include "hardware/encoder.hpp"

#include <micromouse/robot/robot.h>
#include "main.h"

#include <cmath>
#include <numbers>

//
// Constants.
//

// Encoder constants.
static constexpr int8_t ENCODER_MAGNET_POLES = 6;
static constexpr float GEAR_RATIO = 20.f;
static constexpr float ENCODER_TICKS_PER_ROTATION = (ENCODER_MAGNET_POLES * GEAR_RATIO);

static constexpr units::millimeter_t WHEEL_DIAMETER = 25_mm;
static constexpr units::millimeter_t WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER * std::numbers::pi_v<float>);

static constexpr auto ENCODER_TICKS_PER_DISTANCE = (ENCODER_TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE);
// TODO: Measure instead of calculate
static constexpr units::millimeter_t ENCODER_DISTANCE_PER_TICK = (1.f / ENCODER_TICKS_PER_DISTANCE);

//
// Encoder functions.
//

hardware::Drivetrain::EncoderData Encoder::update(uint16_t ticks) {
  const int32_t delta_ticks = calc_delta_ticks(ticks, m_last_ticks);

  m_last_ticks = ticks;

  m_ticks += delta_ticks;
  m_velocity = delta_ticks / ROBOT_UPDATE_PERIOD_S;

  return {.position = m_ticks * ENCODER_DISTANCE_PER_TICK,
          .velocity = (m_velocity / 1_s) * ENCODER_DISTANCE_PER_TICK};
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
