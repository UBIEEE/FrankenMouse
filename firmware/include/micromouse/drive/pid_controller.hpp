#pragma once

namespace drive {

class PIDController {
  // Proportional
  float m_kp;
  // Integral
  float m_ki;
  // Derivative
  float m_kd;

  // Time delta
  const float m_dt;

  const float m_min_integral;
  const float m_max_integral;

  float m_pos_error = 0.f;
  float m_vel_error = 0.f;

  float m_prev_error = 0.f;
  float m_total_error = 0.f;

 public:
  PIDController(float kp,
                float ki,
                float kd,
                float dt,
                float min_integral = -1.f,
                float max_integral = 1.f);

  void set_pid(float kp, float ki, float kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
  }

  float kp() const { return m_kp; }
  float ki() const { return m_ki; }
  float kd() const { return m_kd; }

  enum Term {
    PROPORTIONAL,
    INTEGRAL,
    DERIVATIVE,
  };

  void set_term(Term term, float value) {
    switch (term) {
      case PROPORTIONAL:
        m_kp = value;
        break;
      case INTEGRAL:
        m_ki = value;
        break;
      case DERIVATIVE:
        m_kd = value;
        break;
    }
  }

  float calculate(float measurement, float setpoint);

  void reset();
};

}  // namespace drive
