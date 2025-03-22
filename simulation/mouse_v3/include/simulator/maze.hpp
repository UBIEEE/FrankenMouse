#pragma once

#include <godot_cpp/classes/node.hpp>

#include <godot_cpp/classes/packed_scene.hpp>

#include <simulator/wall.hpp>

namespace godot {

class Maze : public Node {
  GDCLASS(Maze, Node)

  static constexpr int WIDTH = 16;
  static constexpr float CELL_SIZE = 0.18f;
  static constexpr float CELL_HALF_SIZE = CELL_SIZE / 2.f;

 private:
  struct Cell {
    Wall* north = nullptr;
    Wall* east = nullptr;
    Wall* south = nullptr;
    Wall* west = nullptr;

    uint8_t serialize() const;
    void deserialize(uint8_t data);
  };

  enum Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST,
  };

  Cell m_cells[WIDTH][WIDTH] = {};

 protected:
  static void _bind_methods();

  Ref<PackedScene> m_wall_res;
  Ref<PackedScene> m_post_res;

 public:
  Maze();
  ~Maze();

  void _ready() override;
  void _process(double delta) override;

  void open_maze(const String& path);
  void save_maze(const String& path);

 private:
  void add_post(Vector2 position);
  Wall* add_wall(Vector2 position, bool enabled, float angle = 0.f);

  Cell* neighbor_cell(int x, int y, Direction direction);
};

}  // namespace godot
