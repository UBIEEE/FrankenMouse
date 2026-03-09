#pragma once

#include <micromouse_cli/maze/cell.hpp>
#include <micromouse_cli/maze/coordinate.hpp>
#include <cstdint>

class Maze {
 public:
  inline static constexpr uint8_t WIDTH_CELLS = 16;
  inline static constexpr uint16_t TOTAL_CELLS = WIDTH_CELLS * WIDTH_CELLS;

 private:
  Cell m_cells[TOTAL_CELLS];

 public:
  Maze();

  void reset();

  bool is_wall(Coordinate coord, Direction dir) const { return m_cells[coord].is_wall(dir); }
  bool is_exit(Coordinate coord, Direction dir) const { return m_cells[coord].is_exit(dir); }
  void set_wall(Coordinate coord, Direction dir, bool present = true);

  bool is_cell_visited(Coordinate coord) const { return m_cells[coord].is_visited(); }
  void set_cell_visited(Coordinate coord) { m_cells[coord].set_visited(); }

  const Cell& cell(Coordinate coord) const { return m_cells[coord]; }
  const Cell& operator[](Coordinate coord) const { return m_cells[coord]; }

  void set_cell(Coordinate coord, Cell cell);

  static std::optional<Coordinate> neighbor_coordinate(Coordinate coord, Direction direction);
  Cell* neighbor_cell(Coordinate coord, Direction direction);

  std::string to_string(Coordinate current_position) const;

 private:
  void init_boundaries();
};
