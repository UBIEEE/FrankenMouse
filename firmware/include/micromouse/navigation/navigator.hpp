#pragma once

#include <micromouse/drive/drive_controller.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/vision/vision.hpp>
#include <micromouse/solver/solver.hpp>
#include <micromouse/subsystem.hpp>

namespace navigation {

class Navigator : public Subsystem {
  drive::DriveController& m_drive;
  vision::Vision& m_vision;
  Maze& m_maze;

  Solver* m_solver = nullptr;

  float m_start_cell_position_mm = 0.f;
  float m_target_cell_position_mm;

  maze::Coordinate m_position;
  maze::Direction m_direction;

  maze::Coordinate m_next_position;
  maze::Direction m_next_direction;

  maze::CoordinateSpan m_targets;

  bool m_should_sense = false;
  bool m_done = true;

  enum class Move {
    FORWARD,
    FORWARD_STOP,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_AROUND,
    TURN_AROUND_IN_PLACE,
  } m_move;

 public:
  Navigator(drive::DriveController& drive, vision::Vision& vision, Maze& maze)
      : m_drive(drive), m_vision(vision), m_maze(maze) {}

  void periodic() override;

  void reset_position(maze::Coordinate position,
                      maze::Direction direction,
                      float cell_position_mm);

  void search_to(maze::CoordinateSpan targets, Solver& solver);
  // TODO: solve_to() for faster?

  bool is_done() const {
    return false;
    // return m_done;
  }

 private:
  const drive::DriveController::CompletionCallback m_should_sense_callback =
      [this] { m_should_sense = true; };

  const drive::DriveController::CompletionCallback m_done_callback =
      [this] {
        m_done = true;
        // m_position = m_next_position;
        // m_direction = m_next_direction;
      };

  void move(Move move);
};

}  // namespace navigation
