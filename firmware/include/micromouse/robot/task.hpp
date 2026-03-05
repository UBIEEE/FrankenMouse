#pragma once

#include <cstdint>

namespace robot {

enum class Task : uint8_t {
  STOPPED = 0,

  //
  // 1-10: maze tasks.
  //

  MAZE_SEARCH = 1,
  MAZE_SLOW_SOLVE = 2,
  MAZE_FAST_SOLVE = 3,

  //
  // 11-40: test drive tasks.
  //

  TEST_DRIVE_STRAIGHT_FROM_BACK_WALL_TO_SENSE_SPOT = 11,
  TEST_DRIVE_STRAIGHT_ONE_CELL = 12,
  TEST_DRIVE_TURN_RIGHT_FROM_SENSE_SPOT_TO_SENSE_SPOT = 13,
  TEST_DRIVE_TURN_LEFT_FROM_SENSE_SPOT_TO_SENSE_SPOT = 14,
  TEST_DRIVE_TURN_RIGHT_IN_PLACE = 15,
  TEST_DRIVE_TURN_LEFT_IN_PLACE = 16,
  TEST_DRIVE_TURN_180_IN_PLACE = 17,
  TEST_GYRO = 18,
  TEST_DRIVE_STRAIGHT_FOUR_CELLS_FROM_BACK_WALL_WITH_VISION_ALIGN = 19,

  //
  // 41-50: Manual control tasks.
  //

  MANUAL_CHASSIS_SPEEDS = 41,

  //
  // 128+: other
  //

  ARMED = 128,
  ARMED_TRIGGERING,
  ARMED_TRIGGERED,

  VISION_CALIBRATE,
};

const char* task_to_string(Task task);

}  // namespace robot
