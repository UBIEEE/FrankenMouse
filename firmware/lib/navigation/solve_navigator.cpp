#include <micromouse/navigation/solve_navigator.hpp>
#include <micromouse/robot/robot.hpp>
#include <micromouse/robot/cell_positions.hpp>
#include <micromouse/robot/error.hpp>
#include <cassert>
#include "micromouse/drive/motion_runner.hpp"
#include "units/length.h"

#define LOG_PREFIX "[nav2] "
#include <micromouse/logging.hpp>

using namespace navigation;

void SolveNavigator::solve_to(maze::Coordinate position,
                              maze::Direction direction,
                              units::millimeter_t cell_position,
                              maze::CoordinateSpan targets,
                              FinishSolver& solver) {
  m_drive.stop();

  m_solution_steps = solver.solution(position, direction, targets);
  if (m_solution_steps.empty()) {
    Robot::get().error(robot::NavigationErrorCode::MAZE_UNSOLVABLE);
    return;
  }

  m_done = false;

  // Enqueue all motions at once

  SolveRunStep step;
  while (!m_solution_steps.empty()) {
    step = m_solution_steps.front();
    m_solution_steps.pop();

    LogInfo("moving from ({}, {}) to ({}, {}) with move {}", step.start.x(), step.start.y(), step.end.x(),
            step.end.y(), static_cast<int>(step.type));

    move(step, cell_position);
    cell_position = 0_mm;
  }

  // Drive to middle of cell and stop
  m_drive.enqueue_forward(step.end, 0_mm, step.direction, maze::Cell::HALF_WIDTH, false, true);
  // Turn Around
  m_drive.enqueue_stationary_turn(drive::MotionRunner::TurnAngle::CCW_180, [&] { m_done = true; });
}

void SolveNavigator::move(const SolveRunStep& move, units::millimeter_t cell_position /*= 0_mm*/) {
  switch (move.type) {
    using enum SolveRunStep::Type;
    case FORWARD: {
      maze::Coordinate position = move.start;
      for (int i = 0; i < move.num_cells; ++i) {
        move_forward(position, move.direction, cell_position);
        std::optional<maze::Coordinate> next_position = m_maze.neighbor_coordinate(position, move.direction);
        assert(next_position.has_value());
        position = *next_position;
        cell_position = 0_mm;
      }
      break;
    }
    case TURN_LEFT:
      move_turn(false, move.start, move.direction, cell_position);
      break;
    case TURN_RIGHT:
      move_turn(true, move.start, move.direction, cell_position);
      break;
  }
}

void SolveNavigator::move_forward(maze::Coordinate position,
                                  maze::Direction direction,
                                  units::millimeter_t cell_position) {
  assert(cell_position <= maze::Cell::WIDTH);

  if (cell_position < robot::CellPositions::SENSING_SPOT) {
    units::millimeter_t sense_distance = robot::CellPositions::SENSING_SPOT - cell_position;
    m_drive.enqueue_forward(position, cell_position, direction, sense_distance, true, true,
                            std::bind(&SolveNavigator::verify_surroundings, this));
    cell_position += sense_distance;
  }

  units::millimeter_t remaining_distance = maze::Cell::WIDTH - cell_position;
  m_drive.enqueue_forward(position, cell_position, direction, remaining_distance, true, true);
}

void SolveNavigator::move_turn(bool right,
                               maze::Coordinate position,
                               maze::Direction direction,
                               units::millimeter_t cell_position) {
  assert(cell_position < maze::Cell::HALF_WIDTH);

  units::millimeter_t turn_radius = maze::Cell::HALF_WIDTH - cell_position;
  units::millimeter_t turn_90_curve_length = 151.191897105_mm + 10_mm;

  using enum drive::MotionRunner::TurnAngle;
  drive::MotionRunner::TurnAngle angle = right ? CW_90 : CCW_90;
  // m_drive.enqueue_turn(angle, turn_radius);
  m_drive.enqueue_turn_distance(angle, turn_90_curve_length);

  cell_position += maze::Cell::HALF_WIDTH + turn_radius;

  if (cell_position < maze::Cell::WIDTH) {
    units::millimeter_t remaining_distance = maze::Cell::WIDTH - cell_position;
    m_drive.enqueue_forward(position, cell_position, direction, remaining_distance, true, true);
  }
}

void SolveNavigator::verify_surroundings() {
  // Make sure we see what we expect
}
