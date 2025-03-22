#include <godot_cpp/core/class_db.hpp>
#include <simulator/maze.hpp>

#include <godot_cpp/classes/resource_loader.hpp>

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/node3d.hpp>

#include <fstream>

using namespace godot;

uint8_t Maze::Cell::serialize() const {
  uint8_t data = 0;

  if (north->is_enabled()) {
    data |= 0x1;
  }
  if (east->is_enabled()) {
    data |= 0x2;
  }
  if (south->is_enabled()) {
    data |= 0x4;
  }
  if (west->is_enabled()) {
    data |= 0x8;
  }

  return data;
}

void Maze::Cell::deserialize(uint8_t data) {
  north->set_enabled(data & 0x1);
  east->set_enabled(data & 0x2);
  south->set_enabled(data & 0x4);
  west->set_enabled(data & 0x8);
}

void Maze::_bind_methods() {}

Maze::Maze() {}

Maze::~Maze() {}

void Maze::_ready() {
  m_wall_res = ResourceLoader::get_singleton()->load("res://wall.tscn");
  m_post_res = ResourceLoader::get_singleton()->load("res://post.tscn");

  for (int x = 0; x <= WIDTH; ++x) {
    for (int y = 0; y <= WIDTH; ++y) {
      Vector2 position(-x * CELL_SIZE, y * CELL_SIZE);
      add_post(position);
    }
  }

  for (int x = 0; x < WIDTH; ++x) {
    for (int y = 0; y < WIDTH; ++y) {
      Cell& cell = m_cells[x][y];

      // West

      Wall* west_wall =
          add_wall(Vector2(-x * CELL_SIZE, y * CELL_SIZE + CELL_HALF_SIZE),
                   x == 0 || (x == 1 && y == 0), 90.f);
      cell.west = west_wall;

      Cell* west_cell = neighbor_cell(x, y, WEST);
      if (west_cell) {
        west_cell->east = west_wall;
      }

      // North

      Wall* north_wall = add_wall(
          Vector2(-x * CELL_SIZE - CELL_HALF_SIZE, (y + 1) * CELL_SIZE),
          y == WIDTH - 1);
      cell.north = north_wall;

      Cell* north_cell = neighbor_cell(x, y, NORTH);
      if (north_cell) {
        north_cell->south = north_wall;
      }

      // South

      if (y == 0) {
        Wall* south_wall = add_wall(
            Vector2(-x * CELL_SIZE - CELL_HALF_SIZE, y * CELL_SIZE), true);
        cell.south = south_wall;
      }

      // East

      if (x == WIDTH - 1) {
        Wall* east_wall = add_wall(
            Vector2(-(x + 1) * CELL_SIZE, y * CELL_SIZE + CELL_HALF_SIZE), true,
            90.f);
        cell.east = east_wall;
      }
    }
  }
}

void Maze::_process(double delta) {}

void Maze::open_maze(const String& path) {
  std::ifstream file(path.utf8().get_data(), std::ios::binary);
  if (!file)
    return;

  for (int x = 0; x < WIDTH; ++x) {
    for (int y = 0; y < WIDTH; ++y) {
      Cell& cell = m_cells[x][y];

      uint8_t data;
      file.read(reinterpret_cast<char*>(&data), sizeof(data));
      cell.deserialize(data);
    }
  }
}

void Maze::save_maze(const String& path) {
  std::ofstream file(path.utf8().get_data(), std::ios::binary);
  if (!file)
    return;

  for (int x = 0; x < WIDTH; ++x) {
    for (int y = 0; y < WIDTH; ++y) {
      const Cell& cell = m_cells[x][y];

      uint8_t data = cell.serialize();
      file.write(reinterpret_cast<const char*>(&data), sizeof(data));
    }
  }
}

void Maze::add_post(Vector2 position) {
  Node3D* post = static_cast<Node3D*>(m_post_res->instantiate());
  post->set_position(Vector3(position.x, 0.f, position.y));
  add_child(post);
}

Wall* Maze::add_wall(Vector2 position, bool enabled, float angle) {
  Wall* wall = static_cast<Wall*>(m_wall_res->instantiate());
  wall->set_position(Vector3(position.x, 0.f, position.y));
  wall->set_rotation_degrees(Vector3(0.f, angle, 0.f));
  wall->set_enabled(enabled);
  add_child(static_cast<Node3D*>(wall));
  return wall;
}

Maze::Cell* Maze::neighbor_cell(int x, int y, Direction direction) {
  switch (direction) {
    using enum Direction;
    case NORTH:
      ++y;
      break;
    case EAST:
      ++x;
      break;
    case SOUTH:
      --y;
      break;
    case WEST:
      --x;
      break;
  }

  if (x < 0 || x >= WIDTH || y < 0 || y >= WIDTH)
    return nullptr;

  return &m_cells[x][y];
}

