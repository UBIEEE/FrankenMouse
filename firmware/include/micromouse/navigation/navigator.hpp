#pragma once

#include <micromouse/drive/drive_controller.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/solver/solver.hpp>
#include <micromouse/subsystem.hpp>

namespace navigation {

class Navigator : public Subsystem {
  drive::DriveController& m_drive;
  Maze& m_maze;

  Solver* m_solver = nullptr;

  float m_start_cell_position_mm = 0.f;
  float m_target_cell_position_mm;

  maze::Coordinate m_position;
  maze::Direction m_direction;

  maze::CoordinateSpan m_targets;

  bool m_should_sense = false;

  enum class Move {
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_AROUND_FORWARD,
    TURN_AROUND_TURN_LEFT,
    TURN_AROUND_TURN_RIGHT,
  } m_move;

 public:
  Navigator(drive::DriveController& drive, Maze& maze)
      : m_drive(drive), m_maze(maze) {}

  void periodic() override;

  void reset_position(maze::Coordinate position,
                      maze::Direction direction,
                      float cell_position_mm);

  void search_to(maze::CoordinateSpan targets, Solver& solver);
  // TODO: solve_to() for faster?

  bool is_done() const {
    // TODO: When reached target.
    return false;
  }

 private:
  const drive::DriveController::CompletionCallback m_should_sense_callback =
      [this] { m_should_sense = true; };

  void move(Move move);
};

}  // namespace navigation
