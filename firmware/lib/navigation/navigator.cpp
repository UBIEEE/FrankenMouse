#include <micromouse/navigation/navigator.hpp>

#include <cassert>
#include <micromouse/robot_cell_positions.hpp>

using namespace navigation;

void Navigator::periodic() {
  if (m_done || !m_should_sense)
    return;
  m_should_sense = false;

  m_position = m_next_position;
  m_direction = m_next_direction;

  printf("Cell: (%d, %d), Direction: %d\n", m_position.x(), m_position.y(),
         (int)m_direction);

  // The robot's next cell.

  std::optional<maze::Coordinate> new_position =
      m_maze.neighbor_coordinate(m_position, m_direction);
  assert(new_position);

  m_next_position = *new_position;

  bool at_goal = false;
  for (maze::Coordinate target : m_targets) {
    if (m_next_position == target) {
      at_goal = true;
      break;
    }
  }
  if (at_goal) {
    m_next_direction = m_direction;
    move(Move::FORWARD_STOP);
    return;
  }

  // Update the walls.

  bool is_left_wall = m_vision.left_wall();
  bool is_right_wall = m_vision.right_wall();
  bool is_front_wall = m_vision.front_wall();

  Direction left_direction = maze::left_of(m_direction);
  Direction right_direction = maze::right_of(m_direction);
  Direction front_direction = m_direction;

  maze::Cell& cell = m_maze.cell(m_next_position);

  bool was_left_wall = cell.is_wall(left_direction);
  bool was_right_wall = cell.is_wall(right_direction);
  bool was_front_wall = cell.is_wall(front_direction);

  if (was_left_wall && !is_left_wall) {
    printf("Something went wrong, left wall missing\n");
    // assert(false);
  }
  if (was_right_wall && !is_right_wall) {
    printf("Something went wrong, right wall missing\n");
    // assert(false);
  }
  if (was_front_wall && !is_front_wall) {
    printf("Something went wrong, front wall missing\n");
    // assert(false);
  }

  printf("Left: %d, Right: %d, Front: %d\n", (int)is_left_wall,
         (int)is_right_wall, (int)is_front_wall);

  m_maze.set_wall(m_next_position, left_direction, is_left_wall);
  m_maze.set_wall(m_next_position, right_direction, is_right_wall);
  m_maze.set_wall(m_next_position, front_direction, is_front_wall);
  cell.set_visited();

  // Solve the maze, decide where to move to.

  Direction move_direction = m_solver->next(m_next_position, m_targets);
  printf("Move: %d\n", (int)move_direction);

  if (m_direction == move_direction) {
    move(Move::FORWARD);
    m_next_direction = m_direction;
  }
  else if (maze::left_of(m_direction) == move_direction) {
    move(Move::TURN_LEFT);
    m_next_direction = maze::left_of(m_direction);
  }
  else if (maze::right_of(m_direction) == move_direction) {
    move(Move::TURN_RIGHT);
    m_next_direction = maze::right_of(m_direction);
  }
  else { // Opposite
    move(Move::TURN_AROUND);
    m_next_direction = maze::opposite(m_direction);
  }
}

void Navigator::reset_position(maze::Coordinate position,
                               maze::Direction direction,
                               float cell_position_mm) {
  m_position = m_next_position = position;
  m_direction = m_next_direction = direction;
  m_start_cell_position_mm = cell_position_mm;

  m_maze.cell(m_position).set_visited();
}

void Navigator::search_to(maze::CoordinateSpan targets, Solver& solver) {
  m_drive.stop();
  m_targets = targets;
  m_solver = &solver;
  m_done = false;

  float sense_distance =
      RobotCellPositions::SENSING_SPOT_MM - m_start_cell_position_mm;
  float remaining_distance =
      maze::Cell::WIDTH_MM - RobotCellPositions::SENSING_SPOT_MM;

  m_drive.enqueue_forward(sense_distance, true, m_should_sense_callback);
  m_drive.enqueue_forward(remaining_distance, true);

  printf("Cell: x: %d: y: %d: direction: %d\n", m_position.x(), m_position.y(),
         (int)m_direction);

  m_move = Move::FORWARD;
}

void Navigator::move(Move move) {
  m_move = move;

  float sense_distance = RobotCellPositions::SENSING_SPOT_MM;
  float remaining_distance =
      maze::Cell::WIDTH_MM - RobotCellPositions::SENSING_SPOT_MM;

  switch (move) {
    using enum Move;
    case FORWARD:
      m_drive.enqueue_forward(sense_distance, true, m_should_sense_callback);
      m_drive.enqueue_forward(remaining_distance, true);
      break;
    case FORWARD_STOP:
      m_drive.enqueue_forward(maze::Cell::WIDTH_MM, false, m_done_callback);
      m_start_cell_position_mm = 0.f;
      break;
    case TURN_LEFT:
      m_drive.enqueue_forward(remaining_distance, true);
      m_drive.enqueue_turn(drive::DriveController::TurnAngle::CCW_90,
                           RobotCellPositions::SEARCH_TURN_RADIUS_MM,
                           m_should_sense_callback);
      m_drive.enqueue_forward(remaining_distance, true);
      break;
    case TURN_RIGHT:
      m_drive.enqueue_forward(remaining_distance, true);
      m_drive.enqueue_turn(drive::DriveController::TurnAngle::CW_90,
                           RobotCellPositions::SEARCH_TURN_RADIUS_MM,
                           m_should_sense_callback);
      m_drive.enqueue_forward(remaining_distance, true);
      break;
    case TURN_AROUND:
      m_drive.enqueue_forward(maze::Cell::HALF_WIDTH_MM, false);
      m_drive.enqueue_turn(drive::DriveController::TurnAngle::CW_180);
      m_drive.enqueue_forward(
          RobotCellPositions::SENSING_SPOT_MM - maze::Cell::HALF_WIDTH_MM, true,
          m_should_sense_callback);
      m_drive.enqueue_forward(remaining_distance, true);
      break;
    case TURN_AROUND_IN_PLACE:
      // m_drive.enqueue_turn(drive::DriveController::TurnAngle::CW_180);
      break;
  }
}
