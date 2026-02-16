#include <micromouse/drive/drive_controller.hpp>
#include <micromouse/drive/kinematics.hpp>
#include <micromouse/math.hpp>
#include <units/math.h>

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
  const units::second_t linear_time = m_linear_timer->get();
  const units::second_t angular_time = m_angular_timer->get();

  const bool linear_done = m_linear_profile.is_finished(linear_time);
  const bool angular_done = m_angular_profile.is_finished(angular_time);

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

  ChassisSpeeds chassis_speeds = {.linear_velocity = m_linear_profile.sample(linear_time).velocity,
                                  .angular_velocity = m_angular_profile.sample(angular_time).velocity};

  if (chassis_speeds.linear_velocity > 10_mmps &&
      units::math::abs(chassis_speeds.angular_velocity) < 1_deg_per_s && m_vision.left_wall() &&
      m_vision.right_wall()) {
    // align

    const units::millimeter_t left_distance = m_ir_sensors.get_distance(hardware::IRSensors::MID_LEFT);
    const units::millimeter_t right_distance = m_ir_sensors.get_distance(hardware::IRSensors::MID_RIGHT);
    const units::millimeter_t diff = right_distance - left_distance;

    chassis_speeds.angular_velocity += units::degrees_per_second_t{
        m_vision_align_pid.calculate(diff.value(), 0.f)};  // TODO: make this a function of speed
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
      config_linear(m_current_motion->forward.distance, m_current_motion->forward.end_high);
      config_angular(0_deg);
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

void DriveController::enqueue_forward(units::millimeter_t distance,
                                      bool end_high,
                                      CompletionCallback completion_func) {
  Motion motion{.type = Motion::Type::FORWARD, .forward = {}, .completion_func = completion_func};
  motion.forward.distance = distance;
  motion.forward.end_high = end_high;

  m_motions.push(std::move(motion));

  if (m_motion_state == MotionState::NONE)
    m_motion_state = MotionState::IDLE;
}

void DriveController::enqueue_turn(units::millimeter_t leadup_distance,
                                   TurnAngle angle_option,
                                   units::millimeter_t turn_radius,
                                   units::millimeter_t followup_distance,
                                   CompletionCallback completion_func) {
  const units::radian_t angle = units::degree_t{static_cast<float>(int16_t(angle_option))};

  Motion leadup_motion{.type = Motion::Type::FORWARD, .forward = {}};
  leadup_motion.forward.distance = leadup_distance;

  Motion arc_motion{.type = Motion::Type::TURN, .turn = {}};
  arc_motion.turn.angle = angle_option;
  arc_motion.turn.arc_distance = turn_radius * units::math::abs(angle).value();

  Motion followup_motion{.type = Motion::Type::FORWARD, .forward = {}};
  followup_motion.forward.distance = followup_distance;

  bool is_leadup = (leadup_distance != 0_mm);
  bool is_followup = (followup_distance != 0_mm);

  if (is_leadup) {
    m_motions.push(std::move(leadup_motion));
  }

  (is_followup ? followup_motion : arc_motion).completion_func = completion_func;

  m_motions.push(std::move(arc_motion));

  if (is_followup) {
    m_motions.push(std::move(followup_motion));
  }

  if (m_motion_state == MotionState::NONE)
    m_motion_state = MotionState::IDLE;
}

void DriveController::start_arc(Motion& motion) {
  const units::millimeters_per_second_t turn_linear_velocity = m_linear_profile.final_state().velocity;

  config_linear(motion.turn.arc_distance, turn_linear_velocity, turn_linear_velocity);

  const units::degree_t angle = units::degree_t{static_cast<float>(int16_t(motion.turn.angle))};

  const units::second_t total_time = m_linear_profile.total_time();

  units::degrees_per_second_squared_t angular_acceleration = m_speeds.angular_acceleration;
  units::degrees_per_second_t angular_velocity = m_speeds.angular_velocity;

  // Calculate acceleration and velocity to reach target angle in required time.
  if (!float_equals(turn_linear_velocity.value(), 0.f)) {
    angular_acceleration = (4.f * units::math::abs(angle)) / (total_time * total_time);
    angular_velocity = units::math::sqrt(angular_acceleration * angle);
  }

  config_angular(angle, 0_deg_per_s, angular_velocity, angular_acceleration);
}

void DriveController::config_linear(units::millimeter_t distance, bool end_high) {
  const units::millimeters_per_second_t final_velocity = end_high ? m_speeds.linear_velocity : 0_mmps;

  config_linear(distance, final_velocity);
}

void DriveController::config_linear(units::millimeter_t distance,
                                    units::millimeters_per_second_t final_velocity) {
  config_linear(distance, final_velocity, m_speeds.linear_velocity);
}

void DriveController::config_linear(units::millimeter_t distance,
                                    units::millimeters_per_second_t final_velocity,
                                    units::millimeters_per_second_t max_velocity) {
  // The intention of this snippet is to compensate for when the robot
  // overshoots the end of the last motion. This just reduces the distance to
  // travel by the extra distance traveled.
  //
  // This is commented out for now because it messes things up when debugging in
  // simulation (because timer keeps going...)
#if 1
  const units::millimeter_t final_distance = m_linear_profile.sample(m_linear_timer->get()).distance;

  const units::millimeter_t extra_distance = final_distance - m_linear_profile.distance();

  distance -= extra_distance;
#endif

  TrapezoidProfile<units::millimeters>::Constraints constraints(max_velocity, m_speeds.linear_acceleration);
  m_linear_profile.configure(distance, final_velocity, constraints);

  m_linear_timer->reset();
  m_linear_timer->start();
}

void DriveController::config_angular(units::degree_t angle) {
  TrapezoidProfile<units::degrees>::Constraints constraints(m_speeds.angular_velocity,
                                                            m_speeds.angular_acceleration);
  m_angular_profile.configure(angle, m_speeds.angular_velocity, constraints);

  m_angular_timer->reset();
  m_angular_timer->start();
}

void DriveController::config_angular(units::degree_t angle,
                                     units::degrees_per_second_t final_velocity,
                                     units::degrees_per_second_t max_velocity,
                                     units::degrees_per_second_squared_t acceleration) {
  TrapezoidProfile<units::degrees>::Constraints constraints(max_velocity, acceleration);
  m_angular_profile.configure(angle, final_velocity, constraints);

  m_angular_timer->reset();
  m_angular_timer->start();
}
