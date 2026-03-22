#pragma once

#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/direction.hpp>
#include <micromouse/maze/maze.hpp>
#include <queue>

namespace navigation {

struct SolveRunStep {
  enum class Type {
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
  } type;

  maze::Coordinate start;
  maze::Coordinate end;
  maze::Direction direction; // End direction for turns
  int num_cells;
};

// Solver to be used during the final solve phase, needs to determine a complete path.
class FinishSolver {
 protected:
  FinishSolver() = default;

 public:
  virtual ~FinishSolver() = default;

  virtual std::queue<SolveRunStep> solution(maze::Coordinate start_coord,
                                            maze::Direction start_direction,
                                            maze::CoordinateSpan endpoints) = 0;
};

}  // namespace navigation
