#include <micromouse/drive/drive_controller.hpp>

#include <micromouse/drive/kinematics.hpp>
#include <micromouse/math.hpp>

using namespace drive;

void DriveController::reset() {
  m_linear_profile.reset();

  m_linear_timer->stop();
  m_linear_timer->reset();

  m_angular_profile.reset();

  m_angular_timer->stop();
  m_angular_timer->reset();

  m_motions = {};
  m_motion_state = MotionState::NONE;
  m_current_motion = std::nullopt;
}

void DriveController::periodic() {
DO_PERIODIC:
  const float linear_elapsed_s = m_linear_timer->elapsed_s();
  const float angular_elapsed_s = m_angular_timer->elapsed_s();

  const bool linear_done = m_linear_profile.is_done_at(linear_elapsed_s);
  const bool angular_done = m_angular_profile.is_done_at(angular_elapsed_s);

  switch (m_motion_state) {
    using enum MotionState;
    case MOTION:
      process_motion(linear_done, angular_done);
      if (m_motion_state != IDLE)
        break;
      [[fallthrough]];
    case IDLE:
      if (!m_motions.empty()) {
        start_next_motion();
        goto DO_PERIODIC;  // Process the new motion immediately.
      }
      break;
    case NONE:
      reset();
      return;
  }

  ChassisSpeeds chassis_speeds = {
      .linear_velocity_mmps =
          m_linear_profile.sample(linear_elapsed_s).velocity,
      .angular_velocity_dps =
          m_angular_profile.sample(angular_elapsed_s).velocity};

  if (chassis_speeds.linear_velocity_mmps > 10 &&
      std::abs(chassis_speeds.angular_velocity_dps) < 1.f &&
      m_vision.left_wall() && m_vision.right_wall()) {
    // align

    const float left_distance =
        m_ir_sensors.get_distance_mm(hardware::IRSensors::MID_LEFT);
    const float right_distance =
        m_ir_sensors.get_distance_mm(hardware::IRSensors::MID_RIGHT);
    const float diff = right_distance - left_distance;

    chassis_speeds.angular_velocity_dps += m_vision_align_pid.calculate(
        diff, 0.f);  // TODO: make this a function of speed
  }

  m_drivetrain.set_chassis_speeds(chassis_speeds);
}

void DriveController::publish_periodic_feedback() {}

void DriveController::publish_extra_feedback() {}

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

  m_motion_state = MotionState::MOTION;
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
    if (m_current_motion->completion_func) {
      m_current_motion->completion_func();
    }

    m_motion_state = MotionState::IDLE;
  }
}

void DriveController::process_turn(bool linear_done, bool angular_done) {
  (void)linear_done;
  if (angular_done) {
    if (m_current_motion->completion_func) {
      m_current_motion->completion_func();
    }

    m_motion_state = MotionState::IDLE;
  }
}

void DriveController::enqueue_forward(float distance_mm,
                                      bool end_high,
                                      CompletionCallback completion_func) {
  Motion motion{.type = Motion::Type::FORWARD,
                .forward = {},
                .completion_func = completion_func};
  motion.forward.distance = distance_mm;
  motion.forward.end_high = end_high;

  m_motions.push(std::move(motion));

  if (m_motion_state == MotionState::NONE)
    m_motion_state = MotionState::IDLE;
}

void DriveController::enqueue_turn(float leadup_distance_mm,
                                   TurnAngle angle,
                                   float turn_radius_mm,
                                   float followup_distance_mm,
                                   CompletionCallback completion_func) {
  const float angle_deg = float(int16_t(angle));
  const float angle_rad = deg_to_rad(angle_deg);

  Motion leadup_motion{.type = Motion::Type::FORWARD, .forward = {}};
  leadup_motion.forward.distance = leadup_distance_mm;

  Motion arc_motion{.type = Motion::Type::TURN, .turn = {}};
  arc_motion.turn.angle = angle;
  arc_motion.turn.arc_distance_mm = turn_radius_mm * std::abs(angle_rad);

  Motion followup_motion{.type = Motion::Type::FORWARD, .forward = {}};
  followup_motion.forward.distance = followup_distance_mm;

  bool is_leadup = (leadup_distance_mm != 0.f);
  bool is_followup = (followup_distance_mm != 0.f);

  if (is_leadup) {
    m_motions.push(std::move(leadup_motion));
  }

  (is_followup ? followup_motion : arc_motion).completion_func =
      completion_func;

  m_motions.push(std::move(arc_motion));

  if (is_followup) {
    m_motions.push(std::move(followup_motion));
  }

  if (m_motion_state == MotionState::NONE)
    m_motion_state = MotionState::IDLE;
}

void DriveController::start_arc(Motion& motion) {
  const float turn_linear_velocity_mmps = m_linear_profile.final_velocity();

  config_linear(motion.turn.arc_distance_mm, turn_linear_velocity_mmps,
                turn_linear_velocity_mmps);

  const float angle_deg = int16_t(motion.turn.angle);

  const float total_time_s = m_linear_profile.duration_s();

  float angular_acceleration_dps2 = m_speeds.angular_acceleration_dps2;
  float angular_velocity_dps = m_speeds.angular_velocity_dps;

  // Calculate acceleration and velocity to reach target angle in required time.
  if (!float_equals(turn_linear_velocity_mmps, 0.f)) {
    angular_acceleration_dps2 =
        (4.f * std::abs(angle_deg)) / (total_time_s * total_time_s);
    angular_velocity_dps = std::sqrt(angular_acceleration_dps2 * angle_deg);
  }

  config_angular(angle_deg, 0.f, angular_velocity_dps,
                 angular_acceleration_dps2);
}

void DriveController::config_linear(float distance_mm, bool end_high) {
  const float final_velocity_mmps =
      end_high ? m_speeds.linear_velocity_mmps : 0.f;

  config_linear(distance_mm, final_velocity_mmps);
}

void DriveController::config_linear(float distance_mm,
                                    float final_velocity_mmps) {
  config_linear(distance_mm, final_velocity_mmps,
                m_speeds.linear_velocity_mmps);
}

void DriveController::config_linear(float distance_mm,
                                    float final_velocity_mmps,
                                    float max_velocity_mmps) {
  // The intention of this snippet is to compensate for when the robot
  // overshoots the end of the last motion. This just reduces the distance to
  // travel by the extra distance traveled.
  //
  // This is commented out for now because it messes things up when debugging in
  // simulation (because timer keeps going...)
#if 1
  const float final_distance_mm =
      m_linear_profile.sample(m_linear_timer->elapsed_s()).distance;

  const float extra_distance_mm =
      final_distance_mm - m_linear_profile.final_distance();

  distance_mm -= extra_distance_mm;
#endif

  m_linear_profile.configure(distance_mm, final_velocity_mmps,
                             max_velocity_mmps,
                             m_speeds.linear_acceleration_mmps2);

  m_linear_timer->reset();
  m_linear_timer->start();
}

void DriveController::config_angular(float angle_deg) {
  m_angular_profile.configure(angle_deg, m_speeds.angular_velocity_dps,
                              m_speeds.angular_velocity_dps,
                              m_speeds.angular_acceleration_dps2);

  m_angular_timer->reset();
  m_angular_timer->start();
}

void DriveController::config_angular(float angle_deg,
                                     float final_velocity_dps,
                                     float max_velocity_dps,
                                     float acceleration_dps2) {
  m_angular_profile.configure(angle_deg, final_velocity_dps, max_velocity_dps,
                              acceleration_dps2);

  m_angular_timer->reset();
  m_angular_timer->start();
}
