#pragma once

#include <cstdint>

namespace maze {

class MazeDimensions {
 public:
  static constexpr uint8_t WIDTH_CELLS = 16;
  static constexpr uint16_t TOTAL_CELLS = WIDTH_CELLS * WIDTH_CELLS;
};

}  // namespace maze
