#pragma once

#include <cstdint>

class Constants {
public:
  class RobotMeasurements {
  public:
    static constexpr float LENGTH_MM = 100.f;
    static constexpr float WIDTH_MM  = 70.f;

    static constexpr float CENTER_TO_FRONT_MM = 55.f;
    static constexpr float CENTER_TO_BACK_MM  = 45.f;

    // Distance between the left and right wheels.
    static constexpr float TRACK_WIDTH_MM = 50.4f;

    // Angles clockwise from center forwards.
    static constexpr float MID_PHOTOTRANSISTOR_ANGLE_DEG = 45.f;
    static constexpr float FAR_PHOTOTRANSISTOR_ANGLE_DEG = 5.f;
  };

  class Maze {
  public:
    static constexpr uint8_t WIDTH_CELLS  = 16;
    static constexpr uint16_t TOTAL_CELLS = WIDTH_CELLS * WIDTH_CELLS;
  };

  class Cell {
  public:
    static constexpr float WIDTH_MM          = 180.f;
    static constexpr float WALL_THICKNESS_MM = 12.f;
    static constexpr float HALF_WALL_THICKNESS_MM = WALL_THICKNESS_MM / 2.f;

    static constexpr float FREE_WIDTH_MM = WIDTH_MM - WALL_THICKNESS_MM;
  };

  class RobotCellPositions {
  public:
    static constexpr float BACK_WALL_MM =
        (RobotMeasurements::CENTER_TO_BACK_MM + Cell::HALF_WALL_THICKNESS_MM);

    static constexpr float CENTERED_MM = Cell::WIDTH_MM / 2.f;

    static constexpr float SENSING_SPOT_MM = CENTERED_MM + 55.f;

    static constexpr float TURNING_BEGIN_SPOT_MM = CENTERED_MM + 60.f;
    static constexpr float TURNING_END_SPOT_MM = Cell::WIDTH_MM - TURNING_BEGIN_SPOT_MM;
    static constexpr float TURNING_RADIUS_MM = Cell::WIDTH_MM / 2.f + (Cell::WIDTH_MM - Cell::WALL_THICKNESS_MM - TURNING_BEGIN_SPOT_MM);
  };

  class Vision {
  public:
    static constexpr float HAND_TRIGGER_TIME_S = 1.f;
    static constexpr float HAND_TRIGGER_THRESHOLD_MM = 60.f;

    static constexpr float CALIBRATION_TIME_S = 1.f;

    // Wall trigger threshold is determined at runtime during calibration.
  };
};
