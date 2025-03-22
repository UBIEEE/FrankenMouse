#pragma once

#include <cstdint>

namespace maze {

enum class Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

constexpr Direction opposite(Direction dir) {
  return static_cast<Direction>((static_cast<uint8_t>(dir) + 2) % 4);
}

constexpr Direction left_of(Direction dir) {
  return static_cast<Direction>((static_cast<uint8_t>(dir) + 3) % 4);
}

constexpr Direction right_of(Direction dir) {
  return static_cast<Direction>((static_cast<uint8_t>(dir) + 1) % 4);
}

}  // namespace maze

using maze::Direction;
