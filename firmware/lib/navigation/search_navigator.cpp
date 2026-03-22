#include <micromouse/navigation/search_navigator.hpp>
#include <micromouse/robot/robot.hpp>
#include <micromouse/robot/cell_positions.hpp>
#include <micromouse/robot/error.hpp>
#include <cassert>

#define LOG_PREFIX "[nav1] "
#include <micromouse/logging.hpp>

using namespace navigation;

void SearchNavigator::periodic() {
  if (m_done || !m_should_sense)
    return;

  m_should_sense = false;

  m_position = m_next_position;
  m_direction = m_next_direction;

  LogInfo("cell: ({}, {}), direction: {}", m_position.x(), m_position.y(),
          maze::direction_to_string(m_direction));

  // The robot's next cell.

  std::optional<maze::Coordinate> new_position = m_maze.neighbor_coordinate(m_position, m_direction);
  if (!new_position.has_value()) {
    Robot::get().error(robot::NavigationErrorCode::MAZE_EXIT_IN_BOUNDARY);
    return;
  }

  m_next_position = *new_position;

  bool at_goal = false;
  for (maze::Coordinate target : m_targets) {
    if (m_next_position == target) {
      at_goal = true;
      break;
    }
  }
  if (at_goal) {
    m_next_direction = maze::opposite(m_direction);
    move(Move::FINISH);
    return;
  }

  // Update the walls.

  bool is_left_wall_present = m_vision.left_wall();
  bool is_right_wall_present = m_vision.right_wall();
  bool is_front_wall_present = m_vision.front_wall();

  Direction left_direction = maze::left_of(m_direction);
  Direction right_direction = maze::right_of(m_direction);
  Direction front_direction = m_direction;

  maze::Cell& cell = m_maze.cell(m_next_position);

  bool was_left_wall_present = cell.is_wall(left_direction);
  bool was_right_wall_present = cell.is_wall(right_direction);
  bool was_front_wall_present = cell.is_wall(front_direction);

  bool was_left_direction_seen = cell.is_seen(left_direction);
  bool was_right_direction_seen = cell.is_seen(right_direction);
  bool was_front_direction_seen = cell.is_seen(front_direction);

  if (was_left_direction_seen && (was_left_wall_present != is_left_wall_present)) {
    LogError("something went wrong, left wall {}", is_left_wall_present ? "missing" : "appeared");
    Robot::get().error(robot::NavigationErrorCode::MAZE_WALL_INCONSISTENCY);
    return;
  }
  if (was_right_direction_seen && (was_right_wall_present != is_right_wall_present)) {
    LogError("something went wrong, right wall {}", is_right_wall_present ? "missing" : "appeared");
    Robot::get().error(robot::NavigationErrorCode::MAZE_WALL_INCONSISTENCY);
    return;
  }
  if (was_front_direction_seen && (was_front_wall_present != is_front_wall_present)) {
    LogError("something went wrong, front wall {}", is_front_wall_present ? "missing" : "appeared");
    Robot::get().error(robot::NavigationErrorCode::MAZE_WALL_INCONSISTENCY);
    return;
  }

  LogInfo("left: {}, right: {}, front: {}", is_left_wall_present, is_right_wall_present,
          is_front_wall_present);

  m_maze.set_wall(m_next_position, left_direction, is_left_wall_present);
  m_maze.set_wall(m_next_position, right_direction, is_right_wall_present);
  m_maze.set_wall(m_next_position, front_direction, is_front_wall_present);

  m_feedback.publish<feedback::TopicSend::MAZE_CELL>({m_next_position, cell});
  m_feedback.publish<feedback::TopicSend::MAZE_MOUSE_POSITION>(m_next_position);

  // Solve the maze, decide where to move to.

  Direction move_direction = m_solver->next(m_next_position, m_targets);
  LogInfo("move: {}", maze::direction_to_string(move_direction));

  if (m_direction == move_direction) {
    m_next_direction = m_direction;
    move(Move::FORWARD);
  } else if (maze::left_of(m_direction) == move_direction) {
    m_next_direction = maze::left_of(m_direction);
    move(Move::TURN_LEFT);
  } else if (maze::right_of(m_direction) == move_direction) {
    m_next_direction = maze::right_of(m_direction);
    move(Move::TURN_RIGHT);
  } else {  // Opposite
    m_next_direction = maze::opposite(m_direction);
    move(Move::TURN_AROUND);
  }
}

void SearchNavigator::reset_position(maze::Coordinate position,
                                     maze::Direction direction,
                                     units::millimeter_t cell_position) {
  m_position = m_next_position = position;
  m_direction = m_next_direction = direction;
  m_start_cell_position = cell_position;
}

void SearchNavigator::search_to(maze::CoordinateSpan targets, SearchSolver& solver) {
  m_drive.stop();
  m_targets = targets;
  m_solver = &solver;
  m_done = false;

  units::millimeter_t sense_distance = robot::CellPositions::SENSING_SPOT - m_start_cell_position;
  assert(sense_distance >= 0_mm);
  if (sense_distance > 0_mm) {
    m_drive.enqueue_forward(m_position, m_start_cell_position, m_direction, sense_distance,
                            {.end_high = true}, true, m_should_sense_callback);
  } else {
    m_should_sense = true;
  }

  units::millimeter_t remaining_distance = maze::Cell::WIDTH - robot::CellPositions::SENSING_SPOT;
  m_drive.enqueue_forward(m_position, robot::CellPositions::SENSING_SPOT, m_direction, remaining_distance,
                          {.end_high = m_movement_style == MovementStyle::SMOOTH_MOTION}, true);

  LogInfo("cell: ({}, {}), direction: {}", m_position.x(), m_position.y(),
          maze::direction_to_string(m_direction));

  m_move = Move::FORWARD;
}

void SearchNavigator::move(Move move) {
  m_move = move;

  switch (m_movement_style) {
    using enum MovementStyle;
    case SMOOTH_MOTION:
      move_smooth_motion();
      break;
    case START_AND_STOP_MOTION:
      move_start_and_stop_motion();
      break;
  }
}

void SearchNavigator::move_smooth_motion() {
  units::millimeter_t sense_distance = robot::CellPositions::SENSING_SPOT;
  units::millimeter_t remaining_distance = maze::Cell::WIDTH - robot::CellPositions::SENSING_SPOT;
  units::millimeter_t turn_distance = robot::CellPositions::SEARCH_TURN_START;

  // Temporary fix for now.
  units::millimeter_t turn_90_curve_length = 109.194147909_mm + (robot::is_real() ? 8_mm : 0_mm);

  switch (m_move) {
    using enum Move;
    case FORWARD:
      // Drive to sense spot
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, sense_distance, {.end_high = true}, true,
                              m_should_sense_callback);
      // Drive to end of cell
      m_drive.enqueue_forward(m_position, sense_distance, m_direction, remaining_distance, {.end_high = true},
                              true);
      break;
    case FINISH:
      // Drive to middle of cell and stop
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, maze::Cell::HALF_WIDTH, {.end_high = false},
                              true);
      // Turn Around
      m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_180, [&] {
        m_position = m_next_position;
        m_direction = m_next_direction;
        m_start_cell_position = maze::Cell::HALF_WIDTH;
        m_done = true;
      });
      break;
    case TURN_LEFT:
      // Drive to turn start
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, turn_distance, {.end_high = true}, true);
      // Turn and reach sense spot
      // m_drive.enqueue_turn(drive::MotionRunner::TurnAngle::CCW_90,
      // robot::CellPositions::SEARCH_TURN_RADIUS, m_should_sense_callback);
      m_drive.enqueue_turn_distance(drive::MotionRunner::TurnAngle::CCW_90, turn_90_curve_length,
                                    m_should_sense_callback);
      // Drive to end of cell
      m_drive.enqueue_forward(m_position, sense_distance, m_next_direction, remaining_distance,
                              {.end_high = true}, true);
      break;
    case TURN_RIGHT:
      // Drive to turn start
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, turn_distance, {.end_high = true}, true);
      // Turn and reach sense spot
      // m_drive.enqueue_turn(drive::MotionRunner::TurnAngle::CW_90, robot::CellPositions::SEARCH_TURN_RADIUS,
      // m_should_sense_callback);
      m_drive.enqueue_turn_distance(drive::MotionRunner::TurnAngle::CW_90, turn_90_curve_length,
                                    m_should_sense_callback);
      // Drive to end of cell
      m_drive.enqueue_forward(m_position, sense_distance, m_next_direction, remaining_distance,
                              {.end_high = true}, true);
      break;
    case TURN_AROUND:
      // Drive to middle of cell and stop
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, maze::Cell::HALF_WIDTH, {.end_high = false},
                              true);
      // Turn
      m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_180);
      // Drive to sense spot
      m_drive.enqueue_forward(m_position, maze::Cell::HALF_WIDTH, m_next_direction,
                              robot::CellPositions::SENSING_SPOT - maze::Cell::HALF_WIDTH, {.end_high = true},
                              true, m_should_sense_callback);
      // Drive to end of cell
      m_drive.enqueue_forward(m_position, sense_distance, m_next_direction, remaining_distance,
                              {.end_high = true}, true);
      break;
    case TURN_AROUND_IN_PLACE:
      // Already stopped, just turn
      m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_180);
      break;
  }
}

void SearchNavigator::move_start_and_stop_motion() {
  units::millimeter_t sense_distance = robot::CellPositions::SENSING_SPOT;
  units::millimeter_t remaining_distance = maze::Cell::WIDTH - robot::CellPositions::SENSING_SPOT;

  switch (m_move) {
    using enum Move;
    case FORWARD:
      // Drive to sense spot
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, sense_distance, {.end_high = true}, true,
                              m_should_sense_callback);
      // Drive to end of cell
      m_drive.enqueue_forward(m_position, sense_distance, m_direction, remaining_distance,
                              {.end_high = false}, true);
      break;
    case FINISH:
      // Drive to middle of cell and stop
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, maze::Cell::HALF_WIDTH, {.end_high = false},
                              true);
      // Turn Around
      m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_180, [&] {
        m_position = m_next_position;
        m_direction = m_next_direction;
        m_start_cell_position = maze::Cell::HALF_WIDTH;
        m_done = true;
      });
      break;
    case TURN_LEFT:
      // Drive to middle of cell and stop
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, maze::Cell::HALF_WIDTH, {.end_high = false},
                              false);
      // Turn
      m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_90);
      // Drive to sense spot and stop
      m_drive.enqueue_forward(m_position, sense_distance, m_next_direction,
                              robot::CellPositions::SENSING_SPOT - maze::Cell::HALF_WIDTH, {.end_high = true},
                              false, m_should_sense_callback);
      // Drive to end of cell
      m_drive.enqueue_forward(m_position, robot::CellPositions::SENSING_SPOT, m_next_direction,
                              remaining_distance, {.end_high = false}, false);
      break;
    case TURN_RIGHT:
      // Drive to middle of cell and stop
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, maze::Cell::HALF_WIDTH, {.end_high = false},
                              false);
      // Turn
      m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CW_90);
      // Drive to sense spot and stop
      m_drive.enqueue_forward(m_position, sense_distance, m_next_direction,
                              robot::CellPositions::SENSING_SPOT - maze::Cell::HALF_WIDTH, {.end_high = true},
                              false, m_should_sense_callback);
      // Drive to end of cell
      m_drive.enqueue_forward(m_position, robot::CellPositions::SENSING_SPOT, m_next_direction,
                              remaining_distance, {.end_high = false}, false);
      break;
    case TURN_AROUND:
      // Drive to middle of cell and stop
      m_drive.enqueue_forward(m_position, 0_mm, m_direction, maze::Cell::HALF_WIDTH, {.end_high = false},
                              true);
      // Turn
      m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_180);
      // Drive to sense spot
      m_drive.enqueue_forward(m_position, maze::Cell::HALF_WIDTH, m_next_direction,
                              robot::CellPositions::SENSING_SPOT - maze::Cell::HALF_WIDTH, {.end_high = true},
                              true, m_should_sense_callback);
      // Drive to end of cell
      m_drive.enqueue_forward(m_position, sense_distance, m_next_direction, remaining_distance,
                              {.end_high = false}, true);
      break;
    case TURN_AROUND_IN_PLACE:
      // Already stopped, just turn
      m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_180);
      break;
  }
}
