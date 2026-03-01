#pragma once

#include <micromouse/hardware/measurements.hpp>
#include <micromouse/maze/cell.hpp>
#include <units/length.h>

namespace robot {

class CellPositions {
 public:
  static units::millimeter_t back_wall() {
    hardware::RobotMeasurements& m = get_robot_measurements();
    return m.center_to_back + maze::Cell::HALF_WALL_THICKNESS;
  }

  static constexpr units::millimeter_t CENTERED = maze::Cell::WIDTH / 2.f;

  static constexpr units::millimeter_t SENSING_SPOT = CENTERED + 65_mm;

  static constexpr units::millimeter_t SEARCH_TURN_RADIUS =
      maze::Cell::HALF_WIDTH - (maze::Cell::WIDTH - SENSING_SPOT);
};

}  // namespace robot
