#include <micromouse/navigation/solvers/flood_fill.hpp>

#include <cassert>
#include <queue>

using namespace navigation;
using namespace maze;

Direction FloodFillSolver::next(Coordinate robot_coord,
                                CoordinateSpan endpoints,
                                bool should_solve) {
  if (should_solve) {
    solve(endpoints);
  }

  return smallest_neighbor(robot_coord);
}

void FloodFillSolver::solve(CoordinateSpan endpoints) {
  // Eric
}

Direction FloodFillSolver::smallest_neighbor(Coordinate center_coord) const {
  using enum Direction;

  Direction smallest;
  uint8_t smallest_value = 0xFF;

  for (Direction d : {SOUTH, EAST, WEST, NORTH}) {
    if (m_maze.cell(center_coord).is_wall(d))
      continue;

    std::optional<Coordinate> c = m_maze.neighbor_coordinate(center_coord, d);
    if (!c)
      continue;

    uint8_t value = m_cell_values[*c];

    if (value <= smallest_value) {
      smallest = d;
      smallest_value = value;
    }
  }

  return smallest;
}
