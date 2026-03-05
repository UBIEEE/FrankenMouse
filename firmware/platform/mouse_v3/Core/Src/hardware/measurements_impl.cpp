#include <micromouse/hardware/measurements.hpp>

using namespace hardware;

RobotMeasurements& get_robot_measurements() {
  static RobotMeasurements measurements = {
      .length = 100_mm,
      .width = 70_mm,
      .center_to_front = 55_mm,
      .center_to_back = 45_mm,
      .track_width = 60_mm,
      .mid_ir_sensor_angle = 45_deg,
      .far_ir_sensor_angle = 5_deg,
  };

  return measurements;
}
