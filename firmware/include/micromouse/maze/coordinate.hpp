#pragma once

#include <cstdint>
#include <span>
#include <utility>

namespace maze {

class Coordinate {
  uint8_t m_index;

 public:
  Coordinate() : m_index(0) {}

  Coordinate(uint8_t x, uint8_t y);
  explicit Coordinate(uint8_t index);
  explicit Coordinate(std::pair<uint8_t, uint8_t> coord)
      : Coordinate(coord.first, coord.second) {}

  /*implicit*/ operator uint8_t() const { return m_index; }
  explicit operator std::pair<uint8_t, uint8_t>() const { return to_pair(); }

  uint8_t x() const;
  uint8_t y() const;

  std::pair<uint8_t, uint8_t> to_pair() const { return {x(), y()}; }
};

using CoordinateSpan = std::span<const Coordinate>;

}  // namespace maze
