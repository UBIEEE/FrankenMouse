#include "Drive/drive_controller.hpp"

#include "Utilities/math.hpp"

using namespace drive;

static constexpr float VISION_KP = 0.5f;
static constexpr float VISION_KI = 0.f;
static constexpr float VISION_KD = 0.f;

DriveController::DriveController(Drive& drive, Vision& vision)
  : m_drive(drive),
    m_vision(vision),
    m_vision_pid(VISION_KP, VISION_KI, VISION_KD, ROBOT_UPDATE_PERIOD_S) {}

void DriveController::reset() {
  m_linear_profile.reset();
  m_linear_profile.configure(0.f, 0.f, 0.f, 0.f);

  m_linear_timer.stop();
  m_linear_timer.reset();

  m_angular_profile.reset();
  m_angular_profile.configure(0.f, 0.f, 0.f, 0.f);

  m_angular_timer.stop();
  m_angular_timer.reset();

  m_motions      = {};
  m_motion_state = MotionState::NONE;
}

void DriveController::process() {
  const float linear_elapsed_s  = m_linear_timer.elapsed_s();
  const float angular_elapsed_s = m_angular_timer.elapsed_s();

  const bool linear_done  = m_linear_profile.is_done_at(linear_elapsed_s);
  const bool angular_done = m_angular_profile.is_done_at(angular_elapsed_s);

  switch (m_motion_state) {
    using enum MotionState;
  case MOTION:
    process_motion(linear_done, angular_done);
    break;
  case IDLE:
    if (!m_motions.empty()) {
      start_next_motion();
    }
    break;
  case NONE:
    reset();
    return;
  }

  const auto linear_sample  = m_linear_profile.sample(linear_elapsed_s);
  const auto angular_sample = m_angular_profile.sample(angular_elapsed_s);

  float angular_velocity_dps = angular_sample.velocity;

  if (angular_sample.velocity == 0.f) { // Driving straight.
    const float left_reading =
        m_vision.get_wall_distance_mm(Vision::Sensor::MID_LEFT);
    const float right_reading =
        m_vision.get_wall_distance_mm(Vision::Sensor::MID_RIGHT);

    if (left_reading < 100.f && right_reading < 100.f) {
      float vision_diff = right_reading - left_reading;

      angular_velocity_dps += m_vision_pid.calculate(vision_diff, 0.f);
    }
  }

  m_drive.control_speed_velocity(linear_sample.velocity, angular_velocity_dps);
}

void DriveController::start_next_motion() {
  m_current_motion = m_motions.front();
  m_motions.pop();

  switch (m_current_motion->type) {
    using enum Motion::Type;
  case FORWARD:
    config_linear(m_current_motion->forward.distance,
                  m_current_motion->forward.end_high);
    config_angular(0.f);
    break;
  case TURN:
    start_arc(*m_current_motion);
    break;
  }
}

void DriveController::process_motion(bool linear_done, bool angular_done) {
  switch (m_current_motion->type) {
    using enum Motion::Type;
  case FORWARD:
    process_forward(linear_done, angular_done);
    break;
  case TURN:
    process_turn(linear_done, angular_done);
    break;
  }
}

void DriveController::process_forward(bool linear_done, bool angular_done) {
  (void)angular_done;
  if (linear_done) {
    m_motion_state = MotionState::IDLE;
  }
}

void DriveController::process_turn(bool linear_done, bool angular_done) {
  (void)linear_done;
  if (angular_done) {
    m_motion_state = MotionState::IDLE;
  }
}

void DriveController::enqueue_forward(float distance_mm, bool end_high) {
  Motion motion {.type = Motion::Type::FORWARD, .forward = {}};
  motion.forward.distance = distance_mm, motion.forward.end_high = end_high;

  m_motions.push(std::move(motion));
}

void DriveController::enqueue_turn(float leadup_distance_mm, TurnAngle angle,
                                   float followup_distance_mm) {

  const float angle_deg = float(int16_t(angle));
  const float angle_rad = deg_to_rad(angle_deg);

  if (leadup_distance_mm != 0.f) {
    Motion leadup_motion {.type = Motion::Type::FORWARD, .forward = {}};
    leadup_motion.forward.distance = leadup_distance_mm;

    m_motions.push(std::move(leadup_motion));
  }

  {
    Motion arc_motion {.type = Motion::Type::TURN, .turn = {}};
    arc_motion.turn.angle           = angle;
    arc_motion.turn.arc_distance_mm = m_speeds->turn_radius_mm * angle_rad;

    m_motions.push(std::move(arc_motion));
  }

  if (followup_distance_mm != 0.f) {
    Motion followup_motion {.type = Motion::Type::FORWARD, .forward = {}};
    followup_motion.forward.distance = followup_distance_mm;

    m_motions.push(std::move(followup_motion));
  }
}

void DriveController::start_arc(Motion& motion) {
  const float angle_deg = int16_t(motion.turn.angle);

  float turn_linear_velocity_mmps = m_linear_profile.final_velocity();

  float angular_acceleration_dps2 = m_speeds->angular_acceleration_dps2;

  // Calculate acceleration to reach target distance in required time.
  if (!float_equals(turn_linear_velocity_mmps, 0.f)) {
    const float& v_max = m_speeds->angular_velocity_dps;
    const float t_total =
        (motion.turn.arc_distance_mm / turn_linear_velocity_mmps) / 2.f;
    const float x_total = std::abs(angle_deg) / 2.f;

    // x_accel = 0.5 * a * t_accel^2
    // x_cruise = v_max(t_cruise)
    //
    // x_total = x_accel + x_cruise
    // t_total = t_accel + t_cruise
    //
    // v_max = a * t_accel
    // t_accel = v_max / a
    //
    // t_cruise = t_total - t_accel
    // t_cruise = t_total - (v_max / a)
    //
    // x_total = 0.5 * a * t_accel^2 + v * t_cruise
    // x_total = 0.5 * a * (v_max / a)^2 + v_max * (t_total - (v_max / a))
    // x_total = 0.5 * v_max^2 / a + v_max * t_total - v_max^2 / a
    // x_total - v_max * t_total = -0.5 * v_max^2 / a
    // a = -0.5 * v_max^2 / (x_total - v_max * t_total)
    // a = v_max^2 / (2 * (v_max * t_total - x_total))

    const float denom = 2.f * v_max * t_total - x_total;
    // If there is too little time or too large a distance, just use default
    // values and take a little longer to finish turning. Shouldn't ever occur
    // for this robot though, at least I don't think so.
    if (denom > 0) {
      angular_acceleration_dps2 = (v_max * v_max) / denom;
    }
  }

  config_linear(motion.turn.arc_distance_mm, turn_linear_velocity_mmps,
                turn_linear_velocity_mmps);

  config_angular(angle_deg, 0.f, m_speeds->angular_velocity_dps,
                 angular_acceleration_dps2);
}

void DriveController::config_linear(float distance_mm, bool end_high) {
  const float final_velocity_mmps =
      end_high ? m_speeds->linear_velocity_mmps : 0.f;

  config_linear(distance_mm, final_velocity_mmps);
}

void DriveController::config_linear(float distance_mm,
                                    float final_velocity_mmps) {

  config_linear(distance_mm, final_velocity_mmps,
                m_speeds->linear_velocity_mmps);
}

void DriveController::config_linear(float distance_mm,
                                    float final_velocity_mmps,
                                    float max_velocity_mmps) {

  const float final_distance_mm =
      m_linear_profile.sample(m_linear_timer.elapsed_s()).distance;

  const float extra_distance_mm =
      final_distance_mm - m_linear_profile.final_distance();

  distance_mm -= extra_distance_mm;

  m_linear_profile.configure(distance_mm, final_velocity_mmps,
                             max_velocity_mmps,
                             m_speeds->linear_acceleration_mmps2);

  m_linear_timer.reset();
  m_linear_timer.start();
}

void DriveController::config_angular(float angle_deg) {
  m_angular_profile.configure(angle_deg, m_speeds->angular_velocity_dps,
                              m_speeds->angular_velocity_dps,
                              m_speeds->angular_acceleration_dps2);

  m_angular_timer.reset();
  m_angular_timer.start();
}

void DriveController::config_angular(float angle_deg, float final_velocity_dps,
                                     float max_velocity_dps,
                                     float acceleration_dps2) {

  m_angular_profile.configure(angle_deg, final_velocity_dps, max_velocity_dps,
                              acceleration_dps2);

  m_angular_timer.reset();
  m_angular_timer.start();
}
