#pragma once

#include <micromouse/hardware/measurements.hpp>
#include <micromouse/maze/cell.hpp>

class RobotCellPositions {
 public:
  static float back_wall_mm() {
    hardware::RobotMeasurements& m = get_robot_measurements();
    return m.center_to_back_mm + maze::Cell::HALF_WALL_THICKNESS_MM;
  }

  static constexpr float CENTERED_MM = maze::Cell::WIDTH_MM / 2.f;

  static constexpr float SENSING_SPOT_MM = CENTERED_MM + 65.f;

  static constexpr float SEARCH_TURN_RADIUS_MM =
      maze::Cell::HALF_WIDTH_MM - (maze::Cell::WIDTH_MM - SENSING_SPOT_MM);
};
