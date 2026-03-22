#include <micromouse/navigation/solvers/flood_fill.hpp>
#include <queue>
#include <sstream>
#include <cassert>
#include "micromouse/navigation/solvers/finish_solver.hpp"

#define LOG_PREFIX "[flood_fill_solver] "
#include <micromouse/logging.hpp>

using namespace navigation;
using namespace maze;

Direction FloodFillSolver::next(Coordinate robot_coord, CoordinateSpan endpoints, bool should_solve) {
  if (should_solve) {
    solve(endpoints, true);
  }

  LogInfo("maze: \n{}", maze_to_string(robot_coord));

  return smallest_neighbor(robot_coord);
}

std::queue<SolveRunStep> FloodFillSolver::solution(maze::Coordinate start_coord,
                                                   maze::Direction start_direction,
                                                   maze::CoordinateSpan endpoints) {
  solve(endpoints, false);

  std::queue<SolveRunStep> steps;

  Coordinate position = start_coord;
  Direction direction = start_direction;
  bool visited[maze::Maze::TOTAL_CELLS] = {false};
  visited[position] = true;
  bool reached_target = false;
  while (!reached_target) {
    Direction move_direction = smallest_neighbor(position, direction);
    std::optional<Coordinate> new_position_opt = m_maze.neighbor_coordinate(position, move_direction);
    assert(new_position_opt.has_value()); // TODO: Not all these asserts!

    Coordinate new_position = *new_position_opt;
    if (visited[new_position]) {
      // Not solvable?
      return {};
    }

    visited[new_position] = true;

    SolveRunStep::Type type;
    if (move_direction == direction) {
      type = SolveRunStep::Type::FORWARD;
    } else if (maze::left_of(direction) == move_direction) {
      type = SolveRunStep::Type::TURN_LEFT;
    } else if (maze::right_of(direction) == move_direction) {
      type = SolveRunStep::Type::TURN_RIGHT;
    } else {
      assert(false);
    }

    if (type == SolveRunStep::Type::FORWARD && !steps.empty() &&
        steps.back().type == SolveRunStep::Type::FORWARD) {
      SolveRunStep& last_step = steps.back();
      assert(last_step.end == position);
      last_step.end = new_position;
      last_step.num_cells++;
    } else {
      steps.push({.type = type,
                  .start = position,
                  .end = new_position,
                  .direction = move_direction,
                  .num_cells = 1});
    }

    position = new_position;
    direction = move_direction;

    for (maze::Coordinate target : endpoints) {
      if (position == target) {
        reached_target = true;
        break;
      }
    }
  }

  return steps;
}

void FloodFillSolver::solve(CoordinateSpan endpoints, bool dynamic /*= true*/) {
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
      if (!dynamic && !cell.is_seen(d))  // Do not travel through unseen walls if not searching.
        continue;

      if (cell.is_exit(d)) {
        std::optional<Coordinate> next_coord_tmp = m_maze.neighbor_coordinate(coord, d);
        if (!next_coord_tmp.has_value()) {
          LogError("maze exit in boundary at ({}, {}) in direction {}", coord.x(), coord.y(),
                   maze::direction_to_string(d));
          continue;
        }
        Coordinate next_coord = *next_coord_tmp;
        if (m_cell_values[next_coord] > new_value) {
          m_cell_values[next_coord] = new_value;
          queue.push(next_coord);
        }
      }
    }
  }
}

Direction FloodFillSolver::smallest_neighbor(Coordinate center_coord,
                                             std::optional<Direction> preference) const {
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

    if (value == smallest_value && preference.has_value()) {  // Tiebreaker goes to preference
      if (d == *preference || smallest == *preference) {
        smallest = *preference;
      }
    } else if (value <= smallest_value) {
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
