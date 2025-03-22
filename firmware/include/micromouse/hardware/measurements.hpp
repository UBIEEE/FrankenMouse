#pragma once

namespace hardware {

struct RobotMeasurements {
  float length_mm = 100.f;
  float width_mm = 70.f;

  float center_to_front_mm = 55.f;
  float center_to_back_mm = 45.f;

  float track_width_mm = 50.4f;

  // Angles from the forward axis of the robot.
  float mid_ir_sensor_angle_deg = 45.f;
  float far_ir_sensor_angle_deg = 5.f;
};

}  // namespace hardware

hardware::RobotMeasurements& get_robot_measurements();
