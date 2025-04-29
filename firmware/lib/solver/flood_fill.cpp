#include <micromouse/solver/flood_fill.hpp>

#include <cassert>
#include <queue>

using namespace maze;

Direction FloodFillSolver::next(Coordinate robot_coord,
                                CoordinateSpan endpoints,
                                bool should_solve) {
  if (should_solve) {
    solve(endpoints);
  }

  for (int y = MazeDimensions::WIDTH_CELLS - 1; y >= 0; y--) {
    for (int x = 0; x < MazeDimensions::WIDTH_CELLS; x++) {
      Coordinate coord(x, y);
      uint8_t value = m_cell_values[coord];

      if (value < 10) {
        printf(" ");
      }
      printf("%d ", value);
    }
    printf("\n");
  }

  return smallest_neighbor(robot_coord);
}

void FloodFillSolver::solve(CoordinateSpan endpoints) {
  for (uint16_t i = 0; i < MazeDimensions::TOTAL_CELLS; ++i) {
    m_cell_values[i] = 0xFF;
  }

  std::queue<Coordinate> queue;
  for (const Coordinate& endpoint : endpoints) {
    m_cell_values[endpoint] = 0;
    queue.push(endpoint);
  }

  while (!queue.empty()) {
    Coordinate coord = queue.front();
    const Cell& cell = m_maze.cell(coord);

    queue.pop();
    uint8_t new_value = m_cell_values[coord] + 1;

    using enum Direction;
    for (Direction d : {NORTH, EAST, SOUTH, WEST}) {
      if (cell.is_exit(d)) {
        std::optional<Coordinate> next_coord_tmp =
            m_maze.neighbor_coordinate(coord, d);
        assert(next_coord_tmp.has_value());
        Coordinate next_coord = *next_coord_tmp;
        if (m_cell_values[next_coord] > new_value) {
          m_cell_values[next_coord] = new_value;
          queue.push(next_coord);
        }
      }
    }
  }
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
