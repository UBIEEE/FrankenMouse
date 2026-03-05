#include <micromouse/maze/coordinate.hpp>
#include <micromouse/maze/maze.hpp>

namespace maze {

Coordinate::Coordinate(uint8_t x, uint8_t y) : m_index(y * Maze::WIDTH_CELLS + x) {}

Coordinate::Coordinate(uint8_t index) : m_index(index) {}

uint8_t Coordinate::x() const {
  return m_index % Maze::WIDTH_CELLS;
}
uint8_t Coordinate::y() const {
  return m_index / Maze::WIDTH_CELLS;
}

}  // namespace maze
