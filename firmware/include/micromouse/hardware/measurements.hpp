#pragma once

#include <units/length.h>
#include <units/angle.h>

namespace hardware {

struct RobotMeasurements {
  units::millimeter_t length = 100_mm;
  units::millimeter_t width = 70_mm;

  units::millimeter_t center_to_front = 55_mm;
  units::millimeter_t center_to_back = 45_mm;

  units::millimeter_t track_width = 50.4_mm;

  // Angles from the forward axis of the robot.
  units::degree_t mid_ir_sensor_angle = 45_deg;
  units::degree_t far_ir_sensor_angle = 5_deg;
};

}  // namespace hardware

hardware::RobotMeasurements& get_robot_measurements();
