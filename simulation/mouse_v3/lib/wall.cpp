#include <godot_cpp/core/class_db.hpp>
#include <simulator/wall.hpp>

#include <godot_cpp/classes/static_body3d.hpp>

using namespace godot;

void Wall::_bind_methods() {}

Wall::Wall() {}

Wall::~Wall() {}

void Wall::_ready() {
  Area3D* area = get_node<Area3D>("Area3D");

  area->connect("input_event", callable_mp(this, &Wall::input_event));

  StaticBody3D* sb = get_node<StaticBody3D>("StaticBody3D");

  m_collision_shape = sb->get_node<CollisionShape3D>("CollisionShape3D");
  m_mesh = sb->get_node<MeshInstance3D>("MeshInstance3D");

  set_enabled(m_init_enabled);
}

void Wall::_process(double delta) {}

void Wall::input_event(Node* camera,
                       const InputEvent* event,
                       const Vector3& event_position,
                       const Vector3& normal,
                       int shape_idx) {
  if (event->is_pressed()) {
    toggle_enabled();
  }
}

