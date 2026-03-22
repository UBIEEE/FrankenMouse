#pragma once

#include <micromouse/drive/motion_runner.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/vision/vision.hpp>
#include <micromouse/navigation/solvers/finish_solver.hpp>
#include <micromouse/subsystem.hpp>
#include <micromouse/maze/direction.hpp>
#include <units/length.h>
#include <tuple>

namespace navigation {

class SolveNavigator : public Subsystem {
  drive::MotionRunner& m_drive;
  vision::Vision& m_vision;
  hardware::Feedback& m_feedback = get_platform_feedback();

  Maze& m_maze;

  std::queue<SolveRunStep> m_solution_steps;

  maze::Coordinate m_end_position;
  maze::Direction m_end_direction;
  units::millimeter_t m_end_cell_position;

  bool m_done = true;

 public:
  SolveNavigator(drive::MotionRunner& drive, vision::Vision& vision, Maze& maze)
      : m_drive(drive), m_vision(vision), m_maze(maze) {}

  void solve_to(maze::Coordinate position,
                maze::Direction direction,
                units::millimeter_t cell_position,
                maze::CoordinateSpan targets,
                FinishSolver& solver);

  std::tuple<maze::Coordinate, maze::Direction, units::millimeter_t> get_end() const {
    return std::make_tuple(m_end_position, m_end_direction, m_end_cell_position);
  }

  bool is_done() const { return m_done; }

 private:
  void move(const SolveRunStep& move, units::millimeter_t cell_position = 0_mm);
  void move_forward(maze::Coordinate position,
                    maze::Direction direction,
                    units::millimeter_t cell_position,
                    drive::MotionRunner::ForwardMotionEndState end_state);
  void move_turn(bool right,
                 maze::Coordinate position,
                 maze::Direction direction,
                 units::millimeter_t cell_position);

  void verify_surroundings();
};

}  // namespace navigation
