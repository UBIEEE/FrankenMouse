#pragma once

#include <cstdint>

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
      uint8_t visited : 4;
    } m_bits;
  };

 public:
  Cell() : m_data(0x00) {}

  explicit Cell(uint8_t byte) : m_data(byte) {}

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

  void set_north(bool value = true) { m_bits.north = value; }
  void set_east(bool value = true) { m_bits.east = value; }
  void set_south(bool value = true) { m_bits.south = value; }
  void set_west(bool value = true) { m_bits.west = value; }

  bool is_visited() const { return m_bits.visited != 0; }
  void set_visited() noexcept { m_bits.visited = 1; }
};

static_assert(sizeof(Cell) == 1, "Cell must be 1 byte.");
