#pragma once

#include <micromouse/navigation/solvers/search_solver.hpp>
#include <micromouse/navigation/solvers/finish_solver.hpp>

namespace navigation {

class FloodFillSolver : public SearchSolver, public FinishSolver {
  Maze& m_maze;

  uint8_t m_cell_values[maze::Maze::TOTAL_CELLS];

 public:
  FloodFillSolver(Maze& maze) : m_maze(maze) {}

  maze::Direction next(maze::Coordinate coord, maze::CoordinateSpan endpoints, bool solve = true) override;
  std::queue<SolveRunStep> solution(maze::Coordinate start_coord,
                                    maze::Direction start_direction,
                                    maze::CoordinateSpan endpoints) override;

 private:
  void solve(maze::CoordinateSpan endpoints, bool dynamic = true);

  Direction smallest_neighbor(maze::Coordinate coord, std::optional<maze::Direction> preference = std::nullopt) const;

#ifdef WITH_LOGGING
  std::string maze_to_string(maze::Coordinate current_cell) const;
#endif
};

}  // namespace navigation
