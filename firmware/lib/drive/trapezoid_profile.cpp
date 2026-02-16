#if 0
#include <micromouse/drive/trapezoid_profile.hpp>

#include <algorithm>
#include <cmath>

using namespace drive;

void TrapezoidProfile::configure(const State& initial_state,
                                 const State& final_state,
                                 const Constraints& constraints) {
  if (!constraints) {
    reset();
    // TODO: Log error
    return;
  }

  m_direction = get_direction(initial_state, final_state);
  m_initial = direct(initial_state, m_direction);
  m_final = direct(final_state, m_direction);
  m_final_undirected = final_state;
  m_constraints = constraints;

  if (std::abs(m_initial.velocity) > m_constraints.max_velocity) {
    m_initial.velocity = std::copysign(m_constraints.max_velocity, m_initial.velocity);
  }

  float cutoff_begin_time = m_initial.velocity / m_constraints.max_acceleration;
  float cutoff_begin_dist = std::pow(cutoff_begin_time, 2) * m_constraints.max_acceleration / 2.f;

  float cutoff_end_time = m_final.velocity / m_constraints.max_acceleration;
  float cutoff_end_dist = std::pow(cutoff_end_time, 2) * m_constraints.max_acceleration / 2.f;

  float total_dist = cutoff_begin_dist + (m_final.position - m_initial.position) + cutoff_end_dist;
  float accel_time = m_constraints.max_velocity / m_constraints.max_acceleration;

  float cruise_dist = total_dist - std::pow(accel_time, 2) * m_constraints.max_acceleration;

  // Doesn't reach max velocity
  if (cruise_dist < 0.f) {
    accel_time = std::sqrt(total_dist / m_constraints.max_acceleration);
    cruise_dist = 0.f;
  }

  float rise_time = accel_time - cutoff_begin_time;
  float cruise_time = cruise_dist / m_constraints.max_velocity;
  float fall_time = accel_time - cutoff_end_time;

  m_rise_end_time = rise_time;
  m_cruise_end_time = rise_time + cruise_time;
  m_fall_end_time = rise_time + cruise_time + fall_time;

  float rise_dist = (m_initial.velocity + rise_time * m_constraints.max_acceleration / 2.f) * rise_time;
  float fall_dist = (m_final.velocity + fall_time * m_constraints.max_acceleration / 2.f) * fall_time;

  m_rise_end_dist = rise_dist;
  m_cruise_end_dist = rise_dist + cruise_dist;
  m_fall_end_dist = rise_dist + cruise_dist + fall_dist;
}

TrapezoidProfile::SampledState TrapezoidProfile::sample(float t) {
  State result = m_initial;

  if (t < m_rise_end_time) {
    result.velocity += t * m_constraints.max_acceleration;
    result.position += (m_initial.velocity + t * m_constraints.max_acceleration / 2.f) * t;
  } else if (t < m_cruise_end_time) {
    result.velocity = m_constraints.max_velocity;
    result.position += m_rise_end_dist + m_constraints.max_velocity * (t - m_rise_end_time);
  } else if (t < m_fall_end_time) {
    result.velocity = m_constraints.max_velocity - (t - m_cruise_end_time) * m_constraints.max_acceleration;
    float fall_time = t - m_cruise_end_time;
    result.position += m_cruise_end_dist + (m_constraints.max_velocity * fall_time -
                                            m_constraints.max_acceleration * std::pow(fall_time, 2) / 2.f);
  } else {
    result = m_final;
  }

  float distance = result.position - m_initial.position;
  return SampledState(direct(result, m_direction), distance);
}

#endif