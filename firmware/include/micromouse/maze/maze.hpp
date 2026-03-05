#pragma once

#include <cstring>
#include <iterator>
#include <micromouse/maze/cell.hpp>
#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/direction.hpp>
#include <optional>
#include <span>
#include <array>

namespace maze {

/**
 * Represents the maze.
 *
 *   +x is east
 *   +y is north
 *
 *      · · · · ·
 *      · · · · ·
 *      · · t · ·
 *      · · · · ·
 *      r · · · s
 *
 *   _r_ and _s_ are start cells. The mouse starts facing north.
 *   _t_ is the target cell.
 */
class Maze {
 public:
  inline static constexpr uint8_t WIDTH_CELLS = 16;
  inline static constexpr uint16_t TOTAL_CELLS = WIDTH_CELLS * WIDTH_CELLS;

 private:
  Cell m_cells[TOTAL_CELLS];

  // Southwest corner cell (goal is to the right).
  inline static const Coordinate WEST_START {0, 0};
  // Southeast corner cell (goal is to the left).
  inline static const Coordinate EAST_START {15, 0};

  // The cell next to the start cell.
  inline static const Coordinate WEST_OUTSIDE_START {0, 1};
  inline static const Coordinate EAST_OUTSIDE_START {15, 1};

  inline static const std::array<Coordinate, 1> WEST_START_ENDPOINTS = {WEST_START};
  inline static const std::array<Coordinate, 1> EAST_START_ENDPOINTS = {EAST_START};

  inline static const std::array<Coordinate, 1> WEST_OUTSIDE_START_ENDPOINTS = {WEST_OUTSIDE_START};
  inline static const std::array<Coordinate, 1> EAST_OUTSIDE_START_ENDPOINTS = {EAST_OUTSIDE_START};

 public:
  // Center four cells.
  inline static const std::array<Coordinate, 4> GOAL_ENDPOINTS = {Coordinate(7, 7), Coordinate(7, 8),
                                                                  Coordinate(8, 7), Coordinate(8, 8)};

  // The two possible start locations for the mouse.
  enum class StartLocation : uint8_t {
    WEST_OF_GOAL = 0,
    EAST_OF_GOAL = 1,
  };

 public:
  Maze();

  static Coordinate start(StartLocation start_location) {
    switch (start_location) {
      using enum StartLocation;
      case EAST_OF_GOAL:
        return EAST_START;
      case WEST_OF_GOAL:
        return WEST_START;
    }
    return WEST_START;
  }

  static Coordinate outside_start(StartLocation start_location) {
    switch (start_location) {
      using enum StartLocation;
      case EAST_OF_GOAL:
        return EAST_OUTSIDE_START;
      case WEST_OF_GOAL:
        return WEST_OUTSIDE_START;
    }
    return WEST_OUTSIDE_START;
  }

  static CoordinateSpan start_span(StartLocation start_location) {
    switch (start_location) {
      using enum StartLocation;
      case EAST_OF_GOAL:
        return EAST_START_ENDPOINTS;
      case WEST_OF_GOAL:
        return WEST_START_ENDPOINTS;
    }
    return WEST_START_ENDPOINTS;
  }

  static CoordinateSpan outside_start_span(StartLocation start_location) {
    switch (start_location) {
      using enum StartLocation;
      case EAST_OF_GOAL:
        return EAST_OUTSIDE_START_ENDPOINTS;
      case WEST_OF_GOAL:
        return WEST_OUTSIDE_START_ENDPOINTS;
    }
    return WEST_OUTSIDE_START_ENDPOINTS;
  }

  bool operator==(const Maze& other) const {
    return std::memcmp(m_cells, other.m_cells, TOTAL_CELLS * sizeof(Cell)) == 0;
  }
  bool operator!=(const Maze& other) const { return !(*this == other); }

  /**
   * Zeroes out the maze and places boundaries. This does not need to be called initially, just when the maze
   * needs to be reset after being used.
   */
  void reset();

  /**
   * Places a wall bounding the start cell based on the start location.
   *
   * @param start_location The start location of the mouse, which determines where the wall is placed.
   */
  void init_start_cell(StartLocation start_location);

  bool is_wall(Coordinate coord, Direction dir) const { return m_cells[coord].is_wall(dir); }
  bool is_exit(Coordinate coord, Direction dir) const { return m_cells[coord].is_exit(dir); }
  void set_wall(Coordinate coord, Direction dir, bool present = true);

  bool is_cell_visited(Coordinate coord) const { return m_cells[coord].is_visited(); }
  void set_cell_visited(Coordinate coord) { m_cells[coord].set_visited(); }

  Cell& cell(Coordinate coord) { return m_cells[coord]; }
  Cell& operator[](Coordinate coord) { return m_cells[coord]; }
  const Cell& cell(Coordinate coord) const { return m_cells[coord]; }
  const Cell& operator[](Coordinate coord) const { return m_cells[coord]; }

  /**
   * Returns the coordinate of a cell. The cell must be within the maze.
   */
  Coordinate cell_coordinate(const Cell* cell) const { return Coordinate(std::distance(m_cells, cell)); }

  /**
   * Returns the coordinate of the neighbor cell in the specified direction.
   *
   * @param coord The coordinate of the current cell.
   * @param direction The direction of the neighbor cell to get the coordinate of.
   *
   * @return The coordinate, or std::nullopt if the neighbor is out of bounds.
   */
  static std::optional<Coordinate> neighbor_coordinate(Coordinate coord, Direction direction);

  /**
   * Returns a pointer to the neighbor cell in the specified direction.
   *
   * @param coord The coordinate of the current cell.
   * @param direction The direction of the neighbor cell to get.
   *
   * @return The cell, or nullptr if the neighbor is out of bounds.
   */
  Cell* neighbor_cell(Coordinate coord, Direction direction);

 private:
  /**
   * Places walls around the perimeter of the maze.
   */
  void init_boundaries();
};

}  // namespace maze

using maze::Maze;
