#pragma once

#include <micromouse/solver/solver.hpp>

class FloodFillSolver : public Solver {
 public:
  FloodFillSolver(Maze& maze) : Solver(maze) {}

  void solve(maze::CoordinateSpan endpoints) override;
};
