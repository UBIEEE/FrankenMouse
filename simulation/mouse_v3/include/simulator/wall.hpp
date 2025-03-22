#pragma once

#include <godot_cpp/classes/node3d.hpp>

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/input_event.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/static_body3d.hpp>

namespace godot {

class Wall : public Node3D {
  GDCLASS(Wall, Node3D)

  CollisionShape3D* m_collision_shape = nullptr;
  MeshInstance3D* m_mesh = nullptr;

  bool m_init_enabled = true;

 protected:
  static void _bind_methods();

 public:
  Wall();
  ~Wall();

  void _ready() override;
  void _process(double delta) override;

  void set_enabled(bool enabled) {
    if (!m_collision_shape) {
      m_init_enabled = enabled;
      return;
    }
    m_collision_shape->set_disabled(!enabled);
    m_mesh->set_visible(enabled);
  }

  bool is_enabled() const { return !m_collision_shape->is_disabled(); }

  void toggle_enabled() { set_enabled(!is_enabled()); }

 private:
  void input_event(Node* camera,
                   const InputEvent* event,
                   const Vector3& event_position,
                   const Vector3& normal,
                   int shape_idx);
};

}  // namespace godot
