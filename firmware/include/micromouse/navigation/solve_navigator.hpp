#pragma once

#include <micromouse/drive/motion_runner.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/vision/vision.hpp>
#include <micromouse/navigation/solvers/finish_solver.hpp>
#include <micromouse/subsystem.hpp>
#include <micromouse/maze/direction.hpp>
#include <units/length.h>

namespace navigation {

class SolveNavigator : public Subsystem {
  drive::MotionRunner& m_drive;
  vision::Vision& m_vision;
  hardware::Feedback& m_feedback = get_platform_feedback();

  Maze& m_maze;

  std::queue<SolveRunStep> m_solution_steps;

  bool m_done = true;

 public:
  SolveNavigator(drive::MotionRunner& drive, vision::Vision& vision, Maze& maze)
      : m_drive(drive), m_vision(vision), m_maze(maze) {}

  void solve_to(maze::Coordinate position,
                maze::Direction direction,
                units::millimeter_t cell_position,
                maze::CoordinateSpan targets,
                FinishSolver& solver);

  bool is_done() const { return m_done; }

 private:
  void move(const SolveRunStep& move, units::millimeter_t cell_position = 0_mm);
  void move_forward(maze::Coordinate position, maze::Direction direction, units::millimeter_t cell_position);
  void move_turn(bool right,
                 maze::Coordinate position,
                 maze::Direction direction,
                 units::millimeter_t cell_position);

  void verify_surroundings();
};

}  // namespace navigation
