#pragma once

#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/direction.hpp>
#include <micromouse/maze/maze.hpp>

class Solver {
 protected:
  Maze& m_maze;

  Solver(Maze& maze) : m_maze(maze) {}

 public:
  virtual ~Solver() = default;

  /**
   * @brief Solve the maze.
   *
   * @param coord The coordinates of the robot's current cell.
   * @param endpoints The coordinates of the destination cells.
   * @return Direction of the next cell to move to.
   */
  virtual maze::Direction next(maze::Coordinate coord,
                               maze::CoordinateSpan endpoints,
                               bool solve = true) = 0;
};
