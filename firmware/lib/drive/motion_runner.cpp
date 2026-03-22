#include <micromouse/drive/motion_runner.hpp>
#include <micromouse/drive/kinematics.hpp>
#include <micromouse/robot/cell_positions.hpp>
#include <micromouse/vision/vision_distances.hpp>
#include <micromouse/robot/robot.hpp>
#include <micromouse/math.hpp>
#include <micromouse/robot/error.hpp>
#include <micromouse/robot/status_topic.hpp>
#include <units/math.h>
#include <cassert>
#include <numbers>

#define LOG_PREFIX "[drive] "
#include <micromouse/logging.hpp>

namespace drive {

// The turn path is formed by two symmetric segments of an Euler spiral (clothoid).
struct TurnPath {
  units::radian_t angle;
  units::radian_t abs_angle;
  float z;
  float C_z;
  float S_z;

  constexpr TurnPath(units::radian_t angle) : angle(angle), abs_angle(units::math::abs(angle)) {
    z = gcem::sqrt(abs_angle.value() / std::numbers::pi_v<float>);

    // Fresnel integrals
    C_z = approximate_integral(C, 0.0f, z);
    S_z = approximate_integral(S, 0.0f, z);
  }

  constexpr units::millimeter_t length(units::millimeter_t turn_radius) const {
    const float R = turn_radius.value();
    const float half_angle = abs_angle.value() / 2.f;
    float L = (2 * R * z * gcem::tan(half_angle)) / (C_z + S_z * gcem::tan(half_angle));
    return units::millimeter_t{L};
  }

 private:
  static constexpr float S(float u) { return gcem::sin((std::numbers::pi_v<float> / 2.f) * u * u); }
  static constexpr float C(float u) { return gcem::cos((std::numbers::pi_v<float> / 2.f) * u * u); }
};

// Pre-compute turn paths for common angles.
static constexpr TurnPath TURN_CW_90_PATH{-90_deg};
static constexpr TurnPath TURN_CCW_90_PATH{90_deg};

void MotionRunner::reset() {
  m_motion_timer->stop();
  m_motion_timer->reset();

  m_motions = {};
  m_motion_state = MotionState::NONE;
  m_current_motion = nullptr;
}

void MotionRunner::periodic() {
DO_PERIODIC:
  const units::second_t motion_time = m_motion_timer->get();

  ChassisSpeeds chassis_speeds;

  switch (m_motion_state) {
    using enum MotionState;
    case MOTION:
      chassis_speeds = process_motion(motion_time);
      if (m_motion_state != IDLE)
        break;
      [[fallthrough]];  // Motion is done, now idle
    case IDLE:
      if (!m_motions.empty()) {
        start_next_motion(chassis_speeds.linear_velocity);
        goto DO_PERIODIC;  // Process the new motion immediately.
      }
      break;
    case NONE:
      reset();
      return;
  }

  m_drivetrain.set_chassis_speeds(chassis_speeds);
}

void MotionRunner::publish_periodic_feedback() {}

void MotionRunner::publish_status_feedback() {}

void MotionRunner::enqueue_forward(maze::Coordinate start_cell,
                                   units::millimeter_t start_cell_position,
                                   maze::Direction direction,
                                   units::millimeter_t distance,
                                   ForwardMotionEndState end_state,
                                   bool monitor_vision /*= true*/,
                                   CompletionCallback completion_func /*= nullptr*/) {
  auto motion = std::make_unique<ForwardMotion>();
  motion->completion_func = completion_func;
  motion->start_cell = start_cell;
  motion->start_cell_position = start_cell_position;
  motion->direction = direction;
  motion->distance = distance;
  motion->end_state = end_state;
  motion->monitor_vision = monitor_vision;

  m_motions.push(std::move(motion));

  if (m_motion_state == MotionState::NONE) {
    m_motion_state = MotionState::IDLE;
  }
}

void MotionRunner::enqueue_stationary_turn(TurnAngle angle,
                                           CompletionCallback completion_func /*= nullptr*/) {
  enqueue_turn(angle, 0_mm, completion_func);
}

void MotionRunner::enqueue_turn(TurnAngle angle,
                                units::millimeter_t turn_radius,
                                CompletionCallback completion_func /*= nullptr*/) {
  auto motion = std::make_unique<TurnMotion>();
  motion->completion_func = completion_func;
  motion->angle_value = units::degree_t{static_cast<float>(int16_t(angle))};
  if (units::math::abs(motion->angle_value) >= 180_deg) {  // Force 180 degree turns to be stationary.
    turn_radius = 0_mm;
  }
  if (units::math::abs(turn_radius) > 0_mm) {
    switch (angle) {
      using enum TurnAngle;
      case CW_90:
        motion->curve_length = TURN_CW_90_PATH.length(turn_radius);
        break;
      case CCW_90:
        motion->curve_length = TURN_CCW_90_PATH.length(turn_radius);
        break;
      case CW_180:
      case CCW_180:
        assert(false);
        break;
    }
  }

  m_motions.push(std::move(motion));

  if (m_motion_state == MotionState::NONE) {
    m_motion_state = MotionState::IDLE;
  }
}

void MotionRunner::enqueue_turn_distance(TurnAngle angle,
                                         units::millimeter_t curve_length,
                                         CompletionCallback completion_func /*= nullptr*/) {
  auto motion = std::make_unique<TurnMotion>();
  motion->completion_func = completion_func;
  motion->angle_value = units::degree_t{static_cast<float>(int16_t(angle))};
  if (units::math::abs(motion->angle_value) >= 180_deg) {  // Force 180 degree turns to be stationary.
    curve_length = 0_mm;
  }
  motion->curve_length = curve_length;

  m_motions.push(std::move(motion));

  if (m_motion_state == MotionState::NONE) {
    m_motion_state = MotionState::IDLE;
  }
}

void MotionRunner::start_next_motion(units::meters_per_second_t last_velocity) {
  m_current_motion = std::move(m_motions.front());
  m_motions.pop();

  m_motion_state = MotionState::MOTION;

  m_motion_timer->reset();
  m_motion_timer->start();

  switch (m_current_motion->type()) {
    using enum Motion::Type;
    case Motion::Type::FORWARD:
      return start_forward_motion(static_cast<ForwardMotion&>(*m_current_motion), last_velocity);
    case Motion::Type::TURN:
      return start_turn_motion(static_cast<TurnMotion&>(*m_current_motion), last_velocity);
  }
}

void MotionRunner::start_forward_motion(ForwardMotion& motion, units::meters_per_second_t last_velocity) {
  ForwardMotionExecutionProperties& exec = motion.exec_properties;

  exec.initial_velocity = last_velocity;
  exec.current_cell = motion.start_cell;
  exec.current_cell_position = motion.start_cell_position;
  exec.remaining_distance = motion.distance;
  exec.did_vision_adjust_for_current_cell = false;

  using Profile = TrapezoidProfile<units::millimeters>;
  const Profile::Constraints constraints{.max_velocity = m_speeds.linear_velocity,
                                         .max_acceleration = m_speeds.linear_acceleration};
  const Profile::State initial{.position = exec.current_cell_position, .velocity = exec.initial_velocity};
  units::millimeters_per_second_t target_velocity = 0_mmps;
  if (motion.end_state.end_high) {
    target_velocity =
        motion.end_state.end_for_turn ? m_speeds.turn_linear_velocity : m_speeds.linear_velocity;

    if (motion.end_state.distance_until_turn > 0_mm) {
      // If we're going to be turning soon, we may need to start slowing down early to make the turn.
      units::millimeter_t remaining_distance_after_motion =
          motion.end_state.distance_until_turn - exec.remaining_distance;
      const units::millimeters_per_second_t max_velocity_now_for_turn =
          units::math::sqrt(units::math::pow<2>(m_speeds.turn_linear_velocity) +
                            2.f * m_speeds.linear_acceleration * remaining_distance_after_motion);
      target_velocity = std::min(target_velocity, max_velocity_now_for_turn);
    }
  }
  const Profile::State final{.position = exec.current_cell_position + exec.remaining_distance,
                             .velocity = target_velocity};
  exec.linear_profile.configure(initial, final, constraints);
}

void MotionRunner::start_turn_motion(TurnMotion& motion, units::meters_per_second_t last_velocity) {
  TurnMotionExecutionProperties& exec = motion.exec_properties;

  decltype(exec.angular_profile)::Constraints constraints{.max_velocity = m_speeds.angular_velocity,
                                                          .max_acceleration = m_speeds.angular_acceleration};

  if (motion.curve_length > 0_mm && last_velocity > 0_mmps) {
    exec.linear_velocity = last_velocity;

    const units::second_t total_time = motion.curve_length / exec.linear_velocity;

    // Calculate acceleration and velocity to reach target angle in required time.
    const units::degree_t abs_angle = units::math::abs(motion.angle_value);
    constraints.max_acceleration = (4.f * abs_angle) / (total_time * total_time);
    constraints.max_velocity = units::math::sqrt(constraints.max_acceleration * abs_angle);
  } else {
    exec.linear_velocity = 0_mmps;
  }

  exec.angular_profile.configure(motion.angle_value, 0_deg_per_s, 0_deg_per_s, constraints);
}

ChassisSpeeds MotionRunner::process_motion(units::second_t t) {
  switch (m_current_motion->type()) {
    using enum Motion::Type;
    case Motion::Type::FORWARD:
      return process_forward_motion(static_cast<ForwardMotion&>(*m_current_motion), t);
    case Motion::Type::TURN:
      return process_turn_motion(static_cast<TurnMotion&>(*m_current_motion), t);
  }
  return ChassisSpeeds{};
}

ChassisSpeeds MotionRunner::process_forward_motion(ForwardMotion& motion, units::second_t t) {
  ForwardMotionExecutionProperties& exec = motion.exec_properties;

  // Check if done.
  if (t >= exec.linear_profile.total_time()) {
    if (m_current_motion->completion_func) {
      m_current_motion->completion_func();
    }

    m_motion_state = MotionState::IDLE;
    return ChassisSpeeds{.linear_velocity = exec.linear_profile.final_state().velocity,
                         .angular_velocity = 0_deg_per_s};
  }

  const auto& [position, linear_velocity, distance] = exec.linear_profile.sample(t);

  using Profile = TrapezoidProfile<units::millimeters>;

  const Profile::Constraints constraints{.max_velocity = m_speeds.linear_velocity,
                                         .max_acceleration = m_speeds.linear_acceleration};

  // Check if we entered a new cell
  if (exec.current_cell_position + distance > maze::Cell::WIDTH) {
    if (motion.monitor_vision) {  // We only care about cell when we're monitoring vision.
      std::optional<maze::Coordinate> next_cell =
          maze::Maze::neighbor_coordinate(exec.current_cell, motion.direction);
      if (!next_cell.has_value()) {
        Robot::get().error(robot::NavigationErrorCode::MAZE_EXIT_IN_BOUNDARY);
        return ChassisSpeeds{};
      }
      exec.current_cell = *next_cell;
    }
    exec.initial_velocity = linear_velocity;
    exec.current_cell_position += distance - maze::Cell::WIDTH;
    exec.remaining_distance -= distance;
    exec.remaining_distance = std::max(
        0_mm,
        exec.remaining_distance);  // We sometimes overshoot a little when going fast, so just clamp to 0.
    exec.did_vision_adjust_for_current_cell = false;

    const Profile::State initial{.position = exec.current_cell_position, .velocity = exec.initial_velocity};
    units::millimeters_per_second_t target_velocity = 0_mmps;
    if (motion.end_state.end_high) {
      target_velocity =
          motion.end_state.end_for_turn ? m_speeds.turn_linear_velocity : m_speeds.linear_velocity;

      if (motion.end_state.distance_until_turn > 0_mm) {
        // If we're going to be turning soon, we may need to start slowing down early to make the turn.
        units::millimeter_t remaining_distance_after_motion =
            motion.end_state.distance_until_turn - exec.remaining_distance;
        const units::millimeters_per_second_t max_velocity_now_for_turn =
            units::math::sqrt(units::math::pow<2>(m_speeds.turn_linear_velocity) +
                              2.f * m_speeds.linear_acceleration * remaining_distance_after_motion);
        target_velocity = std::min(target_velocity, max_velocity_now_for_turn);
      }
    }
    assert(exec.current_cell_position + exec.remaining_distance > 0_mm);
    const Profile::State final{.position = exec.current_cell_position + exec.remaining_distance,
                               .velocity = target_velocity};
    exec.linear_profile.configure(initial, final, constraints);

    m_motion_timer->reset();
    m_motion_timer->start();
  }

  units::degrees_per_second_t angular_velocity = 0_deg_per_s;

  if (motion.monitor_vision) {
    // Adjust distance traveled when vision sensors tell us where we are.
    if (!exec.did_vision_adjust_for_current_cell) {
      if (m_vision.did_left_wall_just_disappear() || m_vision.did_right_wall_just_disappear()) {
        if (units::math::abs(position - robot::CellPositions::SIDE_WALL_OUT_OF_VIEW_SPOT) < 60_mm) {
          LogInfo("vision adjustment: current position: {} mm, new position: {} mm, diff: {} mm",
                  position.value(), robot::CellPositions::SIDE_WALL_OUT_OF_VIEW_SPOT.value(),
                  (robot::CellPositions::SIDE_WALL_OUT_OF_VIEW_SPOT - position).value());

          Robot::get().feedback_status_update<robot::StatusTopic::MAZE_WALL_GONE>(position.value());

          exec.initial_velocity = linear_velocity;
          units::millimeter_t new_cell_position = robot::CellPositions::SIDE_WALL_OUT_OF_VIEW_SPOT;
          exec.remaining_distance =
              (exec.current_cell_position + exec.remaining_distance) - new_cell_position;
          exec.remaining_distance =
              std::max(0_mm, exec.remaining_distance);  // Lets just hope that it isn't too far off...
                                                        // (previously an error here)
          exec.current_cell_position = new_cell_position;
          exec.did_vision_adjust_for_current_cell = true;

          const Profile::State initial{.position = exec.current_cell_position,
                                       .velocity = exec.initial_velocity};
          units::millimeters_per_second_t target_velocity = 0_mmps;
          if (motion.end_state.end_high) {
            target_velocity =
                motion.end_state.end_for_turn ? m_speeds.turn_linear_velocity : m_speeds.linear_velocity;
          }
          const Profile::State final{.position = exec.current_cell_position + exec.remaining_distance,
                                     .velocity = target_velocity};
          exec.linear_profile.configure(initial, final, constraints);

          m_motion_timer->reset();
          m_motion_timer->start();
        }
      }
    }

    bool vision_align_left_wall = false;
    bool vision_align_right_wall = false;

    if (position < robot::CellPositions::SIDE_WALL_OUT_OF_VIEW_SPOT) {
      const maze::Cell& current_cell = m_maze.cell(exec.current_cell);
      vision_align_left_wall = current_cell.is_wall(maze::left_of(motion.direction));
      vision_align_right_wall = current_cell.is_wall(maze::right_of(motion.direction));
    } else if (!m_maze.is_wall(exec.current_cell, motion.direction)) {
      const maze::Cell* front_cell = m_maze.neighbor_cell(exec.current_cell, motion.direction);
      if (front_cell != nullptr) {
        vision_align_left_wall = front_cell->is_wall(maze::left_of(motion.direction));
        vision_align_right_wall = front_cell->is_wall(maze::right_of(motion.direction));
      }
    }

    vision_align_left_wall &= m_vision.left_wall();
    vision_align_right_wall &= m_vision.right_wall();

    units::millimeter_t alignment_error = 0_mm;

    // Align between both walls.
    if (vision_align_left_wall && vision_align_right_wall) {
      const units::millimeter_t left_distance = m_ir_sensors.get_distance(hardware::IRSensors::MID_LEFT);
      const units::millimeter_t right_distance = m_ir_sensors.get_distance(hardware::IRSensors::MID_RIGHT);
      alignment_error = right_distance - left_distance;
    }
    // Align to single wall.
    else if (vision_align_left_wall || vision_align_right_wall) {
      const hardware::IRSensors::Sensor sensor =
          vision_align_left_wall ? hardware::IRSensors::MID_LEFT : hardware::IRSensors::MID_RIGHT;
      const units::millimeter_t distance = m_ir_sensors.get_distance(sensor);
      alignment_error = distance - vision::VisionDistances::NOMINAL_SIDE_WALL_DISTANCE;
      alignment_error *= vision_align_left_wall ? -1.f : +1.f;
    }

    angular_velocity = units::degrees_per_second_t{
        m_vision_align_pid.calculate(alignment_error.value(), 0.f)};  // TODO: increase at higher speeds?
  }

  return ChassisSpeeds{.linear_velocity = linear_velocity, .angular_velocity = angular_velocity};
}

ChassisSpeeds MotionRunner::process_turn_motion(TurnMotion& motion, units::second_t t) {
  TurnMotionExecutionProperties& exec = motion.exec_properties;

  // Check if done.
  if (t >= exec.angular_profile.total_time()) {
    if (m_current_motion->completion_func) {
      m_current_motion->completion_func();
    }

    m_motion_state = MotionState::IDLE;
    return ChassisSpeeds{.linear_velocity = exec.linear_velocity, .angular_velocity = 0_deg_per_s};
  }

  const auto& [position, angular_velocity, distance] = exec.angular_profile.sample(t);

  return ChassisSpeeds{.linear_velocity = exec.linear_velocity, .angular_velocity = angular_velocity};
}

}  // namespace drive
