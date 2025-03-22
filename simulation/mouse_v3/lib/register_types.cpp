#include <simulator/feedback.hpp>
#include <simulator/maze.hpp>
#include <simulator/menu.hpp>
#include <simulator/micromouse.hpp>
#include <simulator/wall.hpp>

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_simulation_module(ModuleInitializationLevel p_level) {
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
    return;
  }

  GDREGISTER_CLASS(Maze);
  GDREGISTER_CLASS(Wall);

  // Don't run these in the editor.
  GDREGISTER_RUNTIME_CLASS(SimulatorMenu);
  GDREGISTER_RUNTIME_CLASS(MicroMouse);
  GDREGISTER_RUNTIME_CLASS(Feedback);
}

void uninitialize_simulation_module(ModuleInitializationLevel p_level) {
  if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
    return;
  }
}

extern "C" {
// Initialization.
GDExtensionBool GDE_EXPORT
simulation_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
                        const GDExtensionClassLibraryPtr p_library,
                        GDExtensionInitialization* r_initialization) {
  godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library,
                                                 r_initialization);

  init_obj.register_initializer(initialize_simulation_module);
  init_obj.register_terminator(uninitialize_simulation_module);
  init_obj.set_minimum_library_initialization_level(
      MODULE_INITIALIZATION_LEVEL_SCENE);

  return init_obj.init();
}
}

