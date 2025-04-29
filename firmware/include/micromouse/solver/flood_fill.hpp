#pragma once

#include <micromouse/maze/maze_dimensions.hpp>
#include <micromouse/solver/solver.hpp>

class FloodFillSolver : public Solver {
  uint8_t m_cell_values[maze::MazeDimensions::TOTAL_CELLS];

 public:
  FloodFillSolver(Maze& maze) : Solver(maze) {}

  maze::Direction next(maze::Coordinate coord,
                       maze::CoordinateSpan endpoints,
                       bool solve = true) override;

 private:
  void solve(maze::CoordinateSpan endpoints);

  Direction smallest_neighbor(maze::Coordinate coord) const;
};
