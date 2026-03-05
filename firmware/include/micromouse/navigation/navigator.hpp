#pragma once

#include <micromouse/drive/motion_runner.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/vision/vision.hpp>
#include <micromouse/navigation/solvers/solver.hpp>
#include <micromouse/subsystem.hpp>
#include <units/length.h>

namespace navigation {

class Navigator : public Subsystem {
  drive::MotionRunner& m_drive;
  vision::Vision& m_vision;
  Maze& m_maze;

  Solver* m_solver = nullptr;

  units::millimeter_t m_start_cell_position = 0_mm;
  units::millimeter_t m_target_cell_position;

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
  Navigator(drive::MotionRunner& drive, vision::Vision& vision, Maze& maze)
      : m_drive(drive), m_vision(vision), m_maze(maze) {}

  void periodic() override;

  void reset_position(maze::Coordinate position,
                      maze::Direction direction,
                      units::millimeter_t cell_position);

  void search_to(maze::CoordinateSpan targets, Solver& solver);
  // TODO: solve_to() for faster?

  bool is_done() const {
    return false;
    // return m_done;
  }

 private:
  const drive::MotionRunner::CompletionCallback m_should_sense_callback = [this] {
    m_should_sense = true;
  };

  const drive::MotionRunner::CompletionCallback m_done_callback = [this] {
    m_done = true;
    // m_position = m_next_position;
    // m_direction = m_next_direction;
  };

  void move(Move move);
};

}  // namespace navigation
