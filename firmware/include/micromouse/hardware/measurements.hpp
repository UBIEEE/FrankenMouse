#pragma once

#include <units/length.h>
#include <units/angle.h>

namespace hardware {

struct RobotMeasurements {
  units::millimeter_t length = 100_mm;
  units::millimeter_t width = 70_mm;

  /**
   * Distance from the center of the robot to the front and back.
   *
   * The center of the robot is the midpoint between the four wheels, not actually the geometric center of the
   * robot.
   */
  units::millimeter_t center_to_front = 55_mm;
  units::millimeter_t center_to_back = 45_mm;

  // Distance from the center of the left wheel to the center of the right wheel.
  units::millimeter_t track_width = 60_mm;

  // Angles from the forward axis of the robot.
  units::degree_t mid_ir_sensor_angle = 45_deg;
  units::degree_t far_ir_sensor_angle = 5_deg;
};

}  // namespace hardware

hardware::RobotMeasurements& get_robot_measurements();
