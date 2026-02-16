#pragma once

#include <micromouse/maze/direction.hpp>
#include <units/length.h>
#include <cstdint>

namespace maze {

/**
 * @brief Represents a cell in the maze.
 *        Contains data for the walls and whether the cell has been visited.
 *
 * 1 byte in size.
 */
class Cell final {
  union {
    uint8_t m_data;
    struct {
      uint8_t north : 1;
      uint8_t east : 1;
      uint8_t south : 1;
      uint8_t west : 1;
      uint8_t visited : 4;
    } m_bits;
  };

 public:
  static constexpr units::millimeter_t WIDTH = 180_mm;
  static constexpr units::millimeter_t HALF_WIDTH = WIDTH / 2.f;
  static constexpr units::millimeter_t WALL_THICKNESS = 12_mm;
  static constexpr units::millimeter_t HALF_WALL_THICKNESS = WALL_THICKNESS / 2.f;
  static constexpr units::millimeter_t FREE_WIDTH = WIDTH - WALL_THICKNESS;

 public:
  Cell() : m_data(0x00) {}

  Cell(bool north, bool east, bool south, bool west)
      : m_bits{.north = north, .east = east, .south = south, .west = west, .visited = 0} {}

  ~Cell() = default;

  bool operator==(const Cell& other) const { return m_data == other.m_data; }
  bool operator!=(const Cell& other) const { return m_data != other.m_data; }

  void reset() { m_data = 0x00; }

  bool is_north() const { return m_bits.north; }
  bool is_east() const { return m_bits.east; }
  bool is_south() const { return m_bits.south; }
  bool is_west() const { return m_bits.west; }

  bool is_wall(Direction dir) const {
    const uint8_t mask = 1 << static_cast<uint8_t>(dir);
    return (m_data & mask) != 0;
  }

  bool is_exit(Direction dir) const {
    const uint8_t mask = 1 << static_cast<uint8_t>(dir);
    return (m_data & mask) == 0;
  }

  void set_north(bool value = true) { m_bits.north = value; }
  void set_east(bool value = true) { m_bits.east = value; }
  void set_south(bool value = true) { m_bits.south = value; }
  void set_west(bool value = true) { m_bits.west = value; }

  void set_wall(Direction dir, bool value = true) {
    const uint8_t mask = 1 << static_cast<uint8_t>(dir);
    m_data &= ~mask;
    if (value)
      m_data |= mask;
  }

  bool is_visited() const { return m_bits.visited != 0; }
  void set_visited() noexcept { m_bits.visited = 1; }
};

static_assert(sizeof(Cell) == sizeof(uint8_t));

}  // namespace maze
