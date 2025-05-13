#pragma once

#include <cstdint>

enum class Task : uint8_t {
  STOPPED = 0,

  //
  // 1-10: maze tasks.
  //

  MAZE_SEARCH = 1,
  MAZE_SLOW_SOLVE = 2,
  MAZE_FAST_SOLVE = 3,

  //
  // 11-20: test drive tasks.
  //

  // Test drive movements, starting from the back wall.
  TEST_DRIVE_STRAIGHT = 11,
  TEST_DRIVE_LEFT_TURN = 12,
  TEST_DRIVE_RIGHT_TURN = 13,
  TEST_DRIVE_TURN_180 = 14,

  TEST_GYRO = 15,
  TEST_DRIVE_STRAIGHT_VISION_ALIGN = 16,

  //
  // 21-30: Manual control tasks.
  //

  MANUAL_CHASSIS_SPEEDS = 21,

  //
  // 100+: other
  //

  ARMED = 100,
  ARMED_TRIGGERING,
  ARMED_TRIGGERED,

  VISION_CALIBRATE,

  _COUNT,
};
