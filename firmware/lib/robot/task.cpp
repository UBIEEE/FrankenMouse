#include <micromouse/robot/task.hpp>

namespace robot {

const char* task_to_string(Task task) {
  switch (task) {
    using enum Task;
    case STOPPED:
      return "Stopped";
    case MAZE_SEARCH:
      return "[Maze] Search";
    case MAZE_SLOW_SOLVE:
      return "[Maze] Slow Solve";
    case MAZE_FAST_SOLVE:
      return "[Maze] Fast Solve";
    case TEST_DRIVE_STRAIGHT:
      return "[Test] Drive Straight";
    case TEST_DRIVE_LEFT_TURN:
      return "[Test] Drive Left Turn";
    case TEST_DRIVE_RIGHT_TURN:
      return "[Test] Drive Right Turn";
    case TEST_DRIVE_TURN_180:
      return "[Test] Drive Turn 180";
    case TEST_GYRO:
      return "[Test] Gyro";
    case TEST_DRIVE_STRAIGHT_VISION_ALIGN:
      return "[Test] Drive Straight Vision Align";
    case MANUAL_CHASSIS_SPEEDS:
      return "Manual Chassis Speeds";
    case ARMED:
      return "[Armed]";
    case ARMED_TRIGGERING:
      return "[Armed] Triggering";
    case ARMED_TRIGGERED:
      return "[Armed] Triggered";
    case VISION_CALIBRATE:
      return "[Vision] Calibrate";
    case _COUNT:
      break;
  }
  return "Unknown";
}

}  // namespace robot
