#include <micromouse/navigation/navigator.hpp>

#include <cassert>
#include <micromouse/robot_cell_positions.hpp>

using namespace navigation;

void Navigator::periodic() {
  if (!m_should_sense)
    return;
  m_should_sense = false;

  /*
  switch (m_move) {
    using enum Move;
    case FORWARD:
    case TURN_LEFT:
    case TURN_RIGHT: {
      std::optional<maze::Coordinate> new_position =
          m_maze.neighbor_coordinate(m_position, m_direction);
      assert(new_position);

      m_position = *new_position;
      break;
    }
    case TURN_AROUND_FORWARD:
    case TURN_AROUND_TURN_LEFT:
    case TURN_AROUND_TURN_RIGHT:
      m_direction = maze::opposite(m_direction);
      break;
  }

  switch (m_move) {
    using enum Move;
    case TURN_LEFT:
    case TURN_AROUND_TURN_RIGHT:
      m_direction = maze::left_of(m_direction);
      break;
    case TURN_RIGHT:
    case TURN_AROUND_TURN_LEFT:
      m_direction = maze::right_of(m_direction);
      break;
    default:
      break;
  }

  printf("Cell: x: %d: y: %d: direction: %d\n", m_position.x(), m_position.y(),
  (int)m_direction);
  */

  static int x = 0;
  if (x == 0) {
    move(Move::FORWARD);
  } else if (x == 1) {
    move(Move::TURN_RIGHT);
  } else if (x == 2) {
    move(Move::TURN_LEFT);
  } else if (x >= 3 && x < 9) {
    move(Move::FORWARD);
  } else {
    m_drive.enqueue_forward(maze::Cell::WIDTH_MM, false);
  }

  x++;

  // TODO: Update walls...

  m_solver->solve(m_targets);
}

void Navigator::reset_position(maze::Coordinate position,
                               maze::Direction direction,
                               float cell_position_mm) {
  m_position = position;
  m_direction = direction;
  m_start_cell_position_mm = cell_position_mm;
}

void Navigator::search_to(maze::CoordinateSpan targets, Solver& solver) {
  m_drive.stop();
  m_targets = targets;
  m_solver = &solver;

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
    case TURN_AROUND_FORWARD:
      break;
    case TURN_AROUND_TURN_LEFT:
      break;
    case TURN_AROUND_TURN_RIGHT:
      break;
  }
}
