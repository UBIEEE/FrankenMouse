#pragma once

#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/maze.hpp>

class Solver {
 protected:
  Maze& m_maze;

  Solver(Maze& maze) : m_maze(maze) {}

 public:
  virtual ~Solver() = default;

  virtual void solve(maze::CoordinateSpan endpoints) = 0;
};
