#pragma once

#include <micromouse/navigation/solvers/solver.hpp>

namespace navigation {

class FloodFillSolver : public Solver {
  uint8_t m_cell_values[maze::Maze::TOTAL_CELLS];

 public:
  FloodFillSolver(Maze& maze) : Solver(maze) {}

  maze::Direction next(maze::Coordinate coord,
                       maze::CoordinateSpan endpoints,
                       bool solve = true) override;

 private:
  void solve(maze::CoordinateSpan endpoints);

  Direction smallest_neighbor(maze::Coordinate coord) const;

#ifdef WITH_LOGGING
  std::string maze_to_string(maze::Coordinate current_cell) const;
#endif
};

}  // namespace navigation
