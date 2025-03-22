#include <godot_cpp/core/class_db.hpp>
#include <simulator/menu.hpp>

#include <godot_cpp/classes/popup_menu.hpp>

using namespace godot;

void SimulatorMenu::_bind_methods() {}

SimulatorMenu::SimulatorMenu() {}

SimulatorMenu::~SimulatorMenu() {}

void SimulatorMenu::_ready() {
  m_file_dialog = get_node<FileDialog>("FileDialog");
  m_file_dialog->set_use_native_dialog(true);
  m_file_dialog->clear_filters();
  m_file_dialog->add_filter("*.maze", "Maze file");
  m_file_dialog->set_access(FileDialog::ACCESS_FILESYSTEM);
  m_file_dialog->connect("file_selected",
                         callable_mp(this, &SimulatorMenu::file_selected));

  get_popup()->connect("id_pressed",
                       callable_mp(this, &SimulatorMenu::pressed));

  m_maze = get_parent()->get_node<Maze>("Maze");
}

void SimulatorMenu::_process(double delta) {}

void SimulatorMenu::open_maze() {
  m_file_dialog->set_file_mode(FileDialog::FILE_MODE_OPEN_FILE);
  m_file_dialog->show();
}

void SimulatorMenu::save_maze() {
  m_file_dialog->set_file_mode(FileDialog::FILE_MODE_SAVE_FILE);
  m_file_dialog->show();
}

void SimulatorMenu::pressed(int id) {
  switch (id) {
    case ITEM_OPEN:
      open_maze();
      break;
    case ITEM_SAVE:
      save_maze();
      break;
  }
  last_item = static_cast<Item>(id);
}

void SimulatorMenu::file_selected(String path) {
  switch (last_item) {
    case ITEM_OPEN:
      m_maze->open_maze(path);
      break;
    case ITEM_SAVE:
      m_maze->save_maze(path);
      break;
  }
}

