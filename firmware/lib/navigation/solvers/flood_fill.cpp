#include <micromouse/navigation/solvers/flood_fill.hpp>

#include <cassert>
#include <queue>
#include <sstream>

#define LOG_PREFIX "[flood_fill_solver] "
#include <micromouse/logging.hpp>

using namespace navigation;
using namespace maze;

Direction FloodFillSolver::next(Coordinate robot_coord, CoordinateSpan endpoints, bool should_solve) {
  if (should_solve) {
    solve(endpoints);
  }

  LogInfo("maze: \n{}", maze_to_string(robot_coord));

  return smallest_neighbor(robot_coord);
}

void FloodFillSolver::solve(CoordinateSpan endpoints) {
  for (uint16_t i = 0; i < Maze::TOTAL_CELLS; ++i) {
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
        std::optional<Coordinate> next_coord_tmp = m_maze.neighbor_coordinate(coord, d);
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

#ifdef WITH_LOGGING

std::string FloodFillSolver::maze_to_string(maze::Coordinate current_cell) const {
  std::ostringstream out;

  for (int y = Maze::WIDTH_CELLS - 1; y >= 0; --y) {
    for (int x = 0; x < Maze::WIDTH_CELLS; ++x) {
      Coordinate coord(x, y);
      const Cell& cell = m_maze.cell(coord);

      out << (cell.is_wall(Direction::NORTH) ? "+---" : "+   ");
    }

    out << "+\n";

    bool east;
    for (uint16_t x = 0; x < Maze::WIDTH_CELLS; ++x) {
      Coordinate coord(x, y);
      const Cell& cell = m_maze.cell(coord);

      out << (cell.is_wall(Direction::WEST) ? "|" : " ");
      east = cell.is_wall(Direction::EAST);

      if (coord == current_cell) {
        out << "️✈️";
      } else {
        out << ' ';
        uint8_t cell_value = m_cell_values[coord];
        if (cell_value < 10) {
          out << (char)('0' + cell_value);
        } else if ((cell_value - 10) < 26) {
          out << (char)('a' + (cell_value - 10));
        } else if ((cell_value - 36) < 26) {
          out << (char)('A' + (cell_value - 36));
        } else {
          out << 'Z';
        }
      }

      out << ' ';
    }

    out << (east ? "|\n" : " \n");
  }

  for (int x = 0; x < Maze::WIDTH_CELLS; ++x) {
    Coordinate coord(x, 0);
    const Cell& cell = m_maze.cell(coord);
    out << (cell.is_wall(Direction::SOUTH) ? "+---" : "+   ");
  }
  out << "+\n";

  return out.str();
}

#endif
