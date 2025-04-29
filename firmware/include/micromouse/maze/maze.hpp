#pragma once

#include <cstring>
#include <iterator>
#include <micromouse/maze/cell.hpp>
#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/direction.hpp>
#include <micromouse/maze/maze_dimensions.hpp>
#include <optional>
#include <span>

namespace maze {

/**
 * @brief Represents the maze.
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
  Cell m_cells[MazeDimensions::TOTAL_CELLS];

 private:
  // Center four cells.
  inline static constexpr Coordinate GOAL_ENDPOINT_ARRAY[] = {{7, 7},
                                                              {7, 8},
                                                              {8, 7},
                                                              {8, 8}};

  // Southwest corner cell (goal is to the right).
  inline static constexpr Coordinate WEST_START[] = {{0, 0}};
  // Southeast corner cell (goal is to the left).
  inline static constexpr Coordinate EAST_START[] = {{15, 0}};

  // The cell next to the start cell.
  inline static constexpr Coordinate WEST_OUTSIDE_START[] = {{0, 1}};
  inline static constexpr Coordinate EAST_OUTSIDE_START[] = {{15, 1}};

  inline static constexpr CoordinateSpan WEST_START_ENDPOINTS = WEST_START;
  inline static constexpr CoordinateSpan EAST_START_ENDPOINTS = EAST_START;

 public:
  // span of the goal endpoints.
  inline static constexpr CoordinateSpan GOAL_ENDPOINTS = GOAL_ENDPOINT_ARRAY;

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
      case WEST_OF_GOAL:
        return WEST_START[0];
      case EAST_OF_GOAL:
      default:
        return EAST_START[0];
    }
  }

  static Coordinate outside_start(StartLocation start_location) {
    switch (start_location) {
      using enum StartLocation;
      case WEST_OF_GOAL:
        return WEST_OUTSIDE_START[0];
      case EAST_OF_GOAL:
      default:
        return EAST_OUTSIDE_START[0];
    }
  }

  static CoordinateSpan start_span(StartLocation start_location) {
    switch (start_location) {
      using enum StartLocation;
      case WEST_OF_GOAL:
        return WEST_START_ENDPOINTS;
      case EAST_OF_GOAL:
      default:
        return EAST_START_ENDPOINTS;
    }
  }

  static CoordinateSpan outside_start_span(StartLocation start_location) {
    switch (start_location) {
      using enum StartLocation;
      case WEST_OF_GOAL:
        return WEST_OUTSIDE_START;
      case EAST_OF_GOAL:
      default:
        return EAST_OUTSIDE_START;
    }
  }

  bool operator==(const Maze& other) const {
    return std::memcmp(m_cells, other.m_cells, MazeDimensions::TOTAL_CELLS) ==
           0;
  }
  bool operator!=(const Maze& other) const { return !(*this == other); }

  /**
   * @brief  Zeroes out the maze and places boundaries. This does not need to be
   *         called initially, just when the maze needs to be reset after being
   *         used.
   */
  void reset();

  /**
   * @brief Places a wall bounding the start cell based on the start location.
   *
   * @param start_location
   */
  void init_start_cell(StartLocation start_location);

  bool is_wall(Coordinate coord, Direction dir) const {
    return m_cells[coord].is_wall(dir);
  }
  bool is_exit(Coordinate coord, Direction dir) const {
    return m_cells[coord].is_exit(dir);
  }
  void set_wall(Coordinate coord, Direction dir, bool present = true);

  bool is_cell_visited(Coordinate coord) const {
    return m_cells[coord].is_visited();
  }
  void set_cell_visited(Coordinate coord) { m_cells[coord].set_visited(); }

  Cell& cell(Coordinate coord) { return m_cells[coord]; }
  Cell& operator[](Coordinate coord) { return m_cells[coord]; }

  /**
   * @brief Returns the coordinate of a cell. The cell must be within the maze.
   */
  Coordinate cell_coordinate(const Cell* cell) const {
    return Coordinate(std::distance(m_cells, cell));
  }

  /**
   * @brief Returns the coordinate of the neighbor cell in the specified
   *        direction.
   * @param coord
   * @param direction
   * @return std::optional<Coordinate> nullopt if out of bounds.
   */
  std::optional<Coordinate> neighbor_coordinate(Coordinate coord,
                                                Direction direction) const;

  /**
   * @brief Returns a pointer to the neighbor cell in the specified direction.
   *
   * @param coord
   * @param direction
   * @return Cell* nullptr if the neighbor is out of bounds.
   */
  Cell* neighbor_cell(Coordinate coord, Direction direction);

 private:
  /**
   * @brief Places walls around the perimeter of the maze.
   */
  void init_boundaries();
};

}  // namespace maze

using maze::Maze;
