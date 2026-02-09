#pragma once

#include <cstdint>
#include <span>
#include <utility>

class Coordinate {
  uint8_t m_index;

 public:
  Coordinate() : m_index(0) {}

  constexpr Coordinate(uint8_t x, uint8_t y) : m_index(y * 16 + x) {}

  constexpr explicit Coordinate(std::pair<uint8_t, uint8_t> coord) : Coordinate(coord.first, coord.second) {}

  explicit Coordinate(uint8_t index) : m_index(index) {}

  /*implicit*/ operator uint8_t() const { return m_index; }
  explicit operator std::pair<uint8_t, uint8_t>() const { return to_pair(); }

  uint8_t x() const { return m_index % 16; }
  uint8_t y() const { return m_index / 16; }

  std::pair<uint8_t, uint8_t> to_pair() const { return {x(), y()}; }
};

static_assert(sizeof(Coordinate) == 1, "Coordinate must be 1 byte.");

using CoordinateSpan = std::span<const Coordinate>;
