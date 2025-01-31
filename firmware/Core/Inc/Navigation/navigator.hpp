#pragma once

#include "Basic/subsystem.hpp"
#include "Drive/drive_controller.hpp"
#include "Drive/speed_config.hpp"
#include "Maze/coordinate.hpp"
#include "Maze/direction.hpp"
#include "Maze/maze.hpp"
#include "Solver/solver.hpp"
#include <queue>

class Navigator : public Subsystem {
  Maze& m_maze;
  DriveController& m_drive;
  const SpeedConfig& m_speeds;

  float m_start_cell_position_mm = 0.f;
  float m_target_cell_position_mm;

  maze::Coordinate m_position;
  maze::Direction m_direction;

  maze::CoordinateSpan m_target;

  const Solver* m_solver;

public:
  enum class Mode {
    SEARCH,
    SOLVE_SLOW,
    SOLVE_FAST,
  };

private:
  Mode m_mode;

  bool m_done = true;

public:
  Navigator(DriveController& drive, Maze& maze, const SpeedConfig& speeds)
    : m_maze(maze),
      m_drive(drive),
      m_speeds(speeds) {}

  void process() override;
  void on_connect_send_feedback() override;

private:
  void next_motion_slow();
  void next_motion_fast();

public:
  void reset_position(maze::Coordinate position, maze::Direction direction,
                      float cell_position_mm);

  bool is_done() const { return m_done; }
  void stop() { m_done = true; }

  void search_to(maze::CoordinateSpan target, const Solver& solver);
  void solve_to(maze::CoordinateSpan target, bool fast);

private:
  void send_position_direction();
};
