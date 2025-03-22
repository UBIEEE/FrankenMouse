#include <micromouse/drive/pid_controller.hpp>

#include <algorithm>

using namespace drive;

PIDController::PIDController(float kp,
                             float ki,
                             float kd,
                             float dt,
                             float min_integral,
                             float max_integral)
    : m_kp(kp),
      m_ki(ki),
      m_kd(kd),
      m_dt(dt),
      m_min_integral(min_integral),
      m_max_integral(max_integral) {}

float PIDController::calculate(float measurement, float setpoint) {
  m_prev_error = m_pos_error;
  m_pos_error = setpoint - measurement;

  m_vel_error = (m_pos_error - m_prev_error) / m_dt;

  if (m_ki != 0.f) {
    m_total_error = std::clamp(m_total_error + m_pos_error * m_dt,
                               m_min_integral / m_ki, m_max_integral / m_ki);
  }

  return m_kp * m_pos_error + m_ki * m_total_error + m_kd * m_vel_error;
}

void PIDController::reset() {
  m_pos_error = 0.f;
  m_vel_error = 0.f;
  m_prev_error = 0.f;
  m_total_error = 0.f;
}
