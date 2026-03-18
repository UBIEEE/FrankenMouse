#pragma once

#include <micromouse/maze/direction.hpp>
#include <units/length.h>
#include <cstdint>

namespace maze {

/**
 * Represents a cell in the maze.
 * Contains data for the walls and whether the cell has been visited.
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

      uint8_t north_seen : 1;
      uint8_t east_seen : 1;
      uint8_t south_seen : 1;
      uint8_t west_seen : 1;
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

  Cell(bool north, bool east, bool south, bool west) {
    m_bits.north = north;
    m_bits.east = east;
    m_bits.south = south;
    m_bits.west = west;
    m_bits.north_seen = m_bits.east_seen = m_bits.south_seen = m_bits.west_seen = 1;
  }

  ~Cell() = default;

  bool operator==(const Cell& other) const { return m_data == other.m_data; }
  bool operator!=(const Cell& other) const { return m_data != other.m_data; }

  void reset() { m_data = 0x00; }

  bool is_north() const { return m_bits.north; }
  bool is_east() const { return m_bits.east; }
  bool is_south() const { return m_bits.south; }
  bool is_west() const { return m_bits.west; }

  // Returns true if there is a wall in the given direction.
  bool is_wall(Direction dir) const {
    const uint8_t mask = 1 << static_cast<uint8_t>(dir);
    return (m_data & mask) != 0;
  }

  // Returns true if there is no wall in the given direction.
  bool is_exit(Direction dir) const {
    const uint8_t mask = 1 << static_cast<uint8_t>(dir);
    return (m_data & mask) == 0;
  }

  bool is_north_seen() const { return m_bits.north_seen; }
  bool is_east_seen() const { return m_bits.east_seen; }
  bool is_south_seen() const { return m_bits.south_seen; }
  bool is_west_seen() const { return m_bits.west_seen; }

  // Returns true if the robot has looked in the given direction (whether it knows for sure whether there is
  // or isn't a wall in that direction).
  bool is_seen(Direction dir) const {
    const uint8_t mask = 1 << (static_cast<uint8_t>(dir) + 4);
    return (m_data & mask) != 0;
  }

  void set_north(bool value = true) {
    m_bits.north = value;
    m_bits.north_seen = 1;
  }
  void set_east(bool value = true) {
    m_bits.east = value;
    m_bits.east_seen = 1;
  }
  void set_south(bool value = true) {
    m_bits.south = value;
    m_bits.south_seen = 1;
  }
  void set_west(bool value = true) {
    m_bits.west = value;
    m_bits.west_seen = 1;
  }

  void set_wall(Direction dir, bool value = true) {
    const uint8_t mask = 1 << static_cast<uint8_t>(dir);
    m_data &= ~mask;
    if (value) {
      m_data |= mask;
    }
    m_data |= (1 << (static_cast<uint8_t>(dir) + 4));
  }

  bool is_fully_discovered() const {
    return m_bits.north_seen && m_bits.east_seen && m_bits.south_seen && m_bits.west_seen;
  }
};

static_assert(sizeof(Cell) == sizeof(uint8_t));

}  // namespace maze
