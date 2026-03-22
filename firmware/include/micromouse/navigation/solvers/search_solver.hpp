#pragma once

#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/direction.hpp>
#include <micromouse/maze/maze.hpp>

namespace navigation {

// Solver to be used during the search phase, just needs to decide the next cell to move to.
class SearchSolver {
 protected:
  SearchSolver() = default;

 public:
  virtual ~SearchSolver() = default;

  /**
   * Solve the maze.
   *
   * @param coord The coordinates of the robot's current cell.
   * @param endpoints The coordinates of the destination cells.
   * @param solve Whether the solver algorithm should be run. If false, just use previous solve.
   * @return Direction of the next cell to move to.
   */
  virtual maze::Direction next(maze::Coordinate coord, maze::CoordinateSpan endpoints, bool solve = true) = 0;
};

}  // namespace navigation
