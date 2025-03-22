#pragma once

namespace drive {

class TrapezoidProfile {
  float m_velocity_init;
  float m_velocity_cruise;
  float m_velocity_final = 0.f;

  float m_acceleration;

  float m_dist_rise;
  float m_dist_cruise;
  float m_dist_fall;

  float m_dist_total = 0.f;

  float m_time_rise_s;
  float m_time_cruise_s;
  float m_time_fall_s;

  float m_time_total_s;

  bool m_backwards = false;

 public:
  TrapezoidProfile() { configure(0.f, 0.f, 0.f, 0.f); }

  /**
   * @brief Configure the profile.
   *
   * @todo Handle the transition from forwards to backwards better...
   *
   * @param distance The total distance to travel (negative for backwards).
   * @param final_velocity The velocity to (attempt to) reach at the end. Always
   *                       positive.
   * @param max_velocity The absolute maximum velocity to reach during the
   *                     profile. Always positive.
   * @param acceleration Acceleration and deceleration rate. Always positive.
   *
   * @note The starting velocity will be 0 when the profile is first configured
   *       or reset, and the last profile's final velocity otherwise.
   */
  void configure(float distance,
                 float final_velocity,
                 float max_velocity,
                 float acceleration);

  void reset() {
    m_velocity_final = 0.f;
    configure(0.f, 0.f, 0.f, 0.f);
  }

  float duration_s() const { return m_time_total_s; }
  bool is_done_at(float time_s) { return time_s >= m_time_total_s; }

  float final_distance() const { return m_dist_total; }
  float final_velocity() const { return m_velocity_final; }

  struct Sample {
    float distance;
    float velocity;
  };

  Sample sample(float time_s);
};

}  // namespace drive
