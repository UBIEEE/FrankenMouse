#include <godot_cpp/core/class_db.hpp>
#include <simulator/micromouse.hpp>

#include <algorithm>
#include <cassert>
#include <limits>

using namespace godot;

void MicroMouse::_bind_methods() {}

MicroMouse::MicroMouse() {
  std::fill(m_ir_sensor_distances_mm, m_ir_sensor_distances_mm + 4,
            std::numeric_limits<float>::infinity());
}

MicroMouse::~MicroMouse() {}

void MicroMouse::_ready() {
  m_ir_sensors[FAR_LEFT] = get_node<RayCast3D>("Pivot/Far Left");
  m_ir_sensors[MID_LEFT] = get_node<RayCast3D>("Pivot/Mid Left");
  m_ir_sensors[MID_RIGHT] = get_node<RayCast3D>("Pivot/Mid Right");
  m_ir_sensors[FAR_RIGHT] = get_node<RayCast3D>("Pivot/Far Right");

  assert(m_ir_sensors[FAR_LEFT]);
  assert(m_ir_sensors[MID_LEFT]);
  assert(m_ir_sensors[MID_RIGHT]);
  assert(m_ir_sensors[FAR_RIGHT]);

  m_reset_button = get_parent()->get_node<Button>("ResetButton");
  m_reset_button->connect("pressed", callable_mp(this, &MicroMouse::reset));
  assert(m_reset_button);
}

void MicroMouse::_process(double delta) {
  for (int i = 0; i < 4; i++) {
    RayCast3D* s = m_ir_sensors[i];

    bool colliding = s->is_colliding();

    Vector3 sensor_position = s->get_global_position();

    float reading = 0.f;
    float distance_mm = std::numeric_limits<float>::infinity();

    if (colliding) {
      Vector3 collision_point = m_ir_sensors[i]->get_collision_point();

      float distance = sensor_position.distance_to(collision_point);
      distance_mm = distance * 1000.f;

      reading = 1 - (distance / 0.2f);
    }

    m_ir_sensor_readings[i] = reading;
    m_ir_sensor_distances_mm[i] = distance_mm;
  }
}

void MicroMouse::_physics_process(double delta) {
  const float vl = m_linear_velocity - m_track_width * m_angular_velocity / 2.f;
  const float vr = m_linear_velocity + m_track_width * m_angular_velocity / 2.f;

  const float dl = vl * delta;
  const float dr = vr * delta;

  const float dist = (dr + dl) / 2.f;
  const float delta_theta = (dr - dl) / m_track_width;

  Vector3 rotation = get_rotation();

  const float dz = dist * std::cos(rotation.y + (delta_theta / 2.f));
  const float dx = dist * std::sin(rotation.y + (delta_theta / 2.f));

  rotation.y += delta_theta;
  set_rotation(rotation);

  Vector3 slide = Vector3(dx / delta, 0.f, dz / delta);

  set_velocity(slide);
  move_and_slide();
}

void MicroMouse::reset() {
  set_global_position(Vector3(-0.09f, 0.f, 0.051f));
  set_rotation(Vector3(0.f, 0.f, 0.f));
  set_linear_velocity(0.f);
  set_angular_velocity(0.f);
}

