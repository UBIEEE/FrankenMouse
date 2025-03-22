#pragma once

#include <godot_cpp/classes/file_dialog.hpp>
#include <godot_cpp/classes/menu_button.hpp>

#include <simulator/maze.hpp>

namespace godot {

class SimulatorMenu : public MenuButton {
  GDCLASS(SimulatorMenu, MenuButton)

  FileDialog* m_file_dialog = nullptr;
  Maze* m_maze = nullptr;

  enum Item {
    ITEM_OPEN = 0,
    ITEM_SAVE = 1,
  };

  Item last_item;

 protected:
  static void _bind_methods();

 public:
  SimulatorMenu();
  ~SimulatorMenu();

  void _ready() override;
  void _process(double delta) override;

  void open_maze();
  void save_maze();

 private:
  void pressed(int id);
  void file_selected(String path);
};

}  // namespace godot
