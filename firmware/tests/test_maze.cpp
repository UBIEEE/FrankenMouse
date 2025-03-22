#include <micromouse/maze/maze.hpp>
#include <gtest/gtest.h>

using namespace maze;

TEST(Maze, TestDirection) {
  using enum Direction;

  // opposite
  ASSERT_EQ(NORTH, opposite(SOUTH));
  ASSERT_EQ(SOUTH, opposite(NORTH));
  ASSERT_EQ(EAST, opposite(WEST));
  ASSERT_EQ(WEST, opposite(EAST));

  // left_of
  ASSERT_EQ(EAST, left_of(SOUTH));
  ASSERT_EQ(SOUTH, left_of(WEST));
  ASSERT_EQ(WEST, left_of(NORTH));
  ASSERT_EQ(NORTH, left_of(EAST));

  // right_of
  ASSERT_EQ(WEST, right_of(SOUTH));
  ASSERT_EQ(NORTH, right_of(WEST));
  ASSERT_EQ(EAST, right_of(NORTH));
  ASSERT_EQ(SOUTH, right_of(EAST));
}

TEST(Maze, TestCoordinate) {
  Coordinate coord(7, 7);
  ASSERT_EQ(7, coord.x());
  ASSERT_EQ(7, coord.y());

  Coordinate coord2(7, 7);
  ASSERT_EQ(coord, coord2);

  Coordinate coord3(8, 7);
  ASSERT_NE(coord, coord3);

  Coordinate coord4(0, 0);
  ASSERT_EQ(0x00, coord4);

  Coordinate coord5(15, 15);
  ASSERT_EQ(0xFF, coord5);

  Coordinate coord6(7, 8);
  ASSERT_EQ(0x87, coord6);
}

TEST(Maze, TestInit) {
  using enum Direction;

  Maze maze;

  for (uint8_t x = 0; x < MazeDimensions::WIDTH_CELLS; ++x) {
    for (uint8_t y = 0; y < MazeDimensions::WIDTH_CELLS; ++y) {
      const Cell& cell = maze[Coordinate(x, y)];

      const bool is_west = (x == 0);
      ASSERT_EQ(is_west, cell.is_wall(WEST));

      const bool is_east = (x == MazeDimensions::WIDTH_CELLS - 1);
      ASSERT_EQ(is_east, cell.is_wall(EAST));

      const bool is_south = (y == 0);
      ASSERT_EQ(is_south, cell.is_wall(SOUTH));

      const bool is_north = (y == MazeDimensions::WIDTH_CELLS - 1);
      ASSERT_EQ(is_north, cell.is_wall(NORTH));
    }
  }
}

TEST(Maze, TestInitStartCell) {
  using enum Maze::StartLocation;
  using enum Direction;

  Maze maze;

  maze.init_start_cell(WEST_OF_GOAL);
  ASSERT_TRUE(maze[Coordinate(0, 0)].is_wall(EAST));

  maze.init_start_cell(EAST_OF_GOAL);
  ASSERT_TRUE(maze[Coordinate(15, 0)].is_wall(WEST));
}

TEST(Maze, TestReset) {
  using enum Direction;

  const Maze empty_maze;

  Maze maze;
  maze.set_wall(Coordinate(7, 7), NORTH);
  ASSERT_NE(empty_maze, maze);

  maze.reset();
  ASSERT_EQ(empty_maze, maze);
}

TEST(Maze, TestCell) {
  using enum Direction;

  // Init empty.

  const Cell empty_cell;
  ASSERT_FALSE(empty_cell.is_wall(NORTH));
  ASSERT_FALSE(empty_cell.is_wall(EAST));
  ASSERT_FALSE(empty_cell.is_wall(SOUTH));
  ASSERT_FALSE(empty_cell.is_wall(WEST));

  ASSERT_TRUE(empty_cell.is_exit(NORTH));
  ASSERT_TRUE(empty_cell.is_exit(EAST));
  ASSERT_TRUE(empty_cell.is_exit(SOUTH));
  ASSERT_TRUE(empty_cell.is_exit(WEST));

  ASSERT_FALSE(empty_cell.is_visited());

  // Init full.

  const Cell full_cell(true, true, true, true);
  ASSERT_TRUE(full_cell.is_wall(NORTH));
  ASSERT_TRUE(full_cell.is_wall(EAST));
  ASSERT_TRUE(full_cell.is_wall(SOUTH));
  ASSERT_TRUE(full_cell.is_wall(WEST));

  ASSERT_FALSE(full_cell.is_exit(NORTH));
  ASSERT_FALSE(full_cell.is_exit(EAST));
  ASSERT_FALSE(full_cell.is_exit(SOUTH));
  ASSERT_FALSE(full_cell.is_exit(WEST));

  ASSERT_FALSE(full_cell.is_visited());

  Cell cell;

  // Comparison.

  ASSERT_EQ(empty_cell, cell);

  // Set walls.

  cell.set_wall(NORTH);
  ASSERT_TRUE(cell.is_wall(NORTH));
  ASSERT_FALSE(cell.is_exit(NORTH));
  cell.set_wall(EAST);
  ASSERT_TRUE(cell.is_wall(EAST));
  ASSERT_FALSE(cell.is_exit(EAST));
  cell.set_wall(SOUTH);
  ASSERT_TRUE(cell.is_wall(SOUTH));
  ASSERT_FALSE(cell.is_exit(SOUTH));
  cell.set_wall(WEST);
  ASSERT_TRUE(cell.is_wall(WEST));
  ASSERT_FALSE(cell.is_exit(WEST));

  ASSERT_EQ(full_cell, cell);

  // Un-set walls.

  cell.set_wall(NORTH, false);
  ASSERT_FALSE(cell.is_wall(NORTH));
  ASSERT_TRUE(cell.is_exit(NORTH));
  cell.set_wall(EAST, false);
  ASSERT_FALSE(cell.is_wall(EAST));
  ASSERT_TRUE(cell.is_exit(EAST));
  cell.set_wall(SOUTH, false);
  ASSERT_FALSE(cell.is_wall(SOUTH));
  ASSERT_TRUE(cell.is_exit(SOUTH));
  cell.set_wall(WEST, false);
  ASSERT_FALSE(cell.is_wall(WEST));
  ASSERT_TRUE(cell.is_exit(WEST));

  ASSERT_EQ(empty_cell, cell);

  // Set visited.

  cell.set_visited();
  ASSERT_TRUE(cell.is_visited());
}

TEST(Maze, TestGetNeighborCell) {
  using enum Direction;

  Maze maze;

  Cell* neighbor = maze.neighbor_cell(Coordinate(7, 7), NORTH);
  ASSERT_EQ(&maze[Coordinate(7, 8)], neighbor);

  neighbor = maze.neighbor_cell(Coordinate(7, 7), EAST);
  ASSERT_EQ(&maze[Coordinate(8, 7)], neighbor);

  neighbor = maze.neighbor_cell(Coordinate(7, 7), SOUTH);
  ASSERT_EQ(&maze[Coordinate(7, 6)], neighbor);

  neighbor = maze.neighbor_cell(Coordinate(7, 7), WEST);
  ASSERT_EQ(&maze[Coordinate(6, 7)], neighbor);

  neighbor = maze.neighbor_cell(Coordinate(0, 0), WEST);
  ASSERT_EQ(nullptr, neighbor);
}

TEST(Maze, TestSmallestNeighbor) {
  using enum Direction;

  Maze maze;

  // Maze interior
  maze.set_cell_value(Coordinate(7, 7), 0);
  maze.set_cell_value(Coordinate(7, 8), 1);
  maze.set_cell_value(Coordinate(8, 7), 2);
  maze.set_cell_value(Coordinate(7, 6), 3);
  maze.set_cell_value(Coordinate(6, 7), 4);

  ASSERT_EQ(NORTH, maze.smallest_neighbor(Coordinate(7, 7)));

  // Maze corner
  maze.set_cell_value(Coordinate(0, 1), 2);
  maze.set_cell_value(Coordinate(1, 0), 1);

  ASSERT_EQ(EAST, maze.smallest_neighbor(Coordinate(0, 0)));

  // Maze edge
  maze.set_cell_value(Coordinate(15, 14), 2);
  maze.set_cell_value(Coordinate(14, 13), 3);
  maze.set_cell_value(Coordinate(15, 12), 1);

  ASSERT_EQ(SOUTH, maze.smallest_neighbor(Coordinate(15, 13)));
}

TEST(Maze, TestSetWall) {
  using enum Direction;

  Maze maze;

  maze.set_wall(Coordinate(7, 7), NORTH);
  ASSERT_TRUE(maze[Coordinate(7, 7)].is_wall(NORTH));
  ASSERT_TRUE(maze[Coordinate(7, 8)].is_wall(SOUTH));

  maze.set_wall(Coordinate(7, 7), EAST);
  ASSERT_TRUE(maze[Coordinate(7, 7)].is_wall(EAST));
  ASSERT_TRUE(maze[Coordinate(8, 7)].is_wall(WEST));

  maze.set_wall(Coordinate(7, 7), SOUTH);
  ASSERT_TRUE(maze[Coordinate(7, 7)].is_wall(SOUTH));
  ASSERT_TRUE(maze[Coordinate(7, 6)].is_wall(NORTH));

  maze.set_wall(Coordinate(7, 7), WEST);
  ASSERT_TRUE(maze[Coordinate(7, 7)].is_wall(WEST));
  ASSERT_TRUE(maze[Coordinate(6, 7)].is_wall(EAST));
}
