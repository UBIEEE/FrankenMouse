#pragma once

#include <micromouse/drive/motion_runner.hpp>
#include <micromouse/maze/maze.hpp>
#include <micromouse/vision/vision.hpp>
#include <micromouse/navigation/solvers/search_solver.hpp>
#include <micromouse/subsystem.hpp>
#include <units/length.h>

namespace navigation {

class SearchNavigator : public Subsystem {
  drive::MotionRunner& m_drive;
  vision::Vision& m_vision;
  hardware::Feedback& m_feedback = get_platform_feedback();

  Maze& m_maze;

  SearchSolver* m_solver = nullptr;

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
    FINISH,  // Turns around and stops
    TURN_LEFT,
    TURN_RIGHT,
    TURN_AROUND,
    TURN_AROUND_AND_REVERSE,
    TURN_AROUND_IN_PLACE,
  } m_move;

 public:
  SearchNavigator(drive::MotionRunner& drive, vision::Vision& vision, Maze& maze)
      : m_drive(drive), m_vision(vision), m_maze(maze) {}

  void periodic() override;

  void reset_position(maze::Coordinate position,
                      maze::Direction direction,
                      units::millimeter_t cell_position);

  enum class MovementStyle {
    SMOOTH_MOTION,
    START_AND_STOP_MOTION,
  };

  void set_movement_style(MovementStyle style) { m_movement_style = style; }

  void search_to(maze::CoordinateSpan targets, SearchSolver& solver);

  bool is_done() const { return m_done; }

 private:
  const drive::MotionRunner::CompletionCallback m_should_sense_callback = [this] { m_should_sense = true; };

  MovementStyle m_movement_style = MovementStyle::SMOOTH_MOTION;

  void move(Move move);
  void move_smooth_motion();
  void move_start_and_stop_motion();
};

}  // namespace navigation
