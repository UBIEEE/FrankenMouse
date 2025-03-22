#pragma once

#include <godot_cpp/classes/button.hpp>
#include <godot_cpp/classes/character_body3d.hpp>
#include <godot_cpp/classes/ray_cast3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace godot {

class MicroMouse : public CharacterBody3D {
  GDCLASS(MicroMouse, CharacterBody3D)

  enum {
    FAR_LEFT = 0,
    MID_LEFT = 1,
    MID_RIGHT = 2,
    FAR_RIGHT = 3,
  };

  RayCast3D* m_ir_sensors[4];
  float m_ir_sensor_readings[4];
  float m_ir_sensor_distances_mm[4];

  Button* m_reset_button = nullptr;

 private:
  const float m_track_width = 0.0504f;  // m

  float m_angular_velocity = 0.f;  // rad/s
  float m_linear_velocity = 0.f;   // m/s

 protected:
  static void _bind_methods();

 public:
  MicroMouse();
  ~MicroMouse();

  void _ready() override;
  void _process(double delta) override;
  void _physics_process(double delta) override;

  void set_angular_velocity(float angular_velocity_dps) {
    m_angular_velocity = UtilityFunctions::deg_to_rad(angular_velocity_dps);
  }
  float get_angular_velocity() const {
    return UtilityFunctions::rad_to_deg(m_angular_velocity);
  }

  void set_linear_velocity(float linear_velocity_mps) {
    m_linear_velocity = linear_velocity_mps;
  }
  float get_linear_velocity() const { return m_linear_velocity; }

  const float* get_ir_sensor_readings() const { return m_ir_sensor_readings; }
  const float* get_ir_sensor_distances_mm() const {
    return m_ir_sensor_distances_mm;
  }

 private:
  void reset();
};

}  // namespace godot
