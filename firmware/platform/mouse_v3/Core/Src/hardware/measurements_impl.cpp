#include <micromouse/hardware/measurements.hpp>

using namespace hardware;

RobotMeasurements& get_robot_measurements() {
  static RobotMeasurements measurements = {
      .length_mm = 100.f,
      .width_mm = 70.f,
      .center_to_front_mm = 55.f,
      .center_to_back_mm = 45.f,
      .track_width_mm = 50.4f,
      .mid_ir_sensor_angle_deg = 45.f,
      .far_ir_sensor_angle_deg = 5.f,
  };

  return measurements;
}
