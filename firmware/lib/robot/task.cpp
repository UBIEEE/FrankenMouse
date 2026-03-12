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
    case MAZE_SEARCH_START_STOP_MOTION:
      return "[Maze] Search (Start+Stop Motion)";
    case TEST_DRIVE_STRAIGHT_FROM_BACK_WALL_TO_SENSE_SPOT:
      return "[Test] Drive Straight from Back Wall to Sense Spot";
    case TEST_DRIVE_STRAIGHT_ONE_CELL:
      return "[Test] Drive Straight One Cell";
    case TEST_DRIVE_TURN_RIGHT_FROM_SENSE_SPOT_TO_SENSE_SPOT:
      return "[Test] Drive Turn Right from Sense Spot to Sense Spot";
    case TEST_DRIVE_TURN_LEFT_FROM_SENSE_SPOT_TO_SENSE_SPOT:
      return "[Test] Drive Turn Left from Sense Spot to Sense Spot";
    case TEST_DRIVE_TURN_RIGHT_IN_PLACE:
      return "[Test] Drive Turn Right In Place";
    case TEST_DRIVE_TURN_LEFT_IN_PLACE:
      return "[Test] Drive Turn Left In Place";
    case TEST_DRIVE_TURN_180_IN_PLACE:
      return "[Test] Drive Turn 180 In Place";
    case TEST_GYRO:
      return "[Test] Gyro";
    case TEST_DRIVE_STRAIGHT_FOUR_CELLS_FROM_BACK_WALL_WITH_VISION_ALIGN:
      return "[Test] Drive Straight Four Cells from Back Wall with Vision Align";
    case MANUAL_CHASSIS_SPEEDS:
      return "Manual Chassis Speeds";
    case IDLE:
      return "Idle";
    case ARMED:
      return "[Armed]";
    case ARMED_TRIGGERING:
      return "[Armed] Triggering";
    case ARMED_TRIGGERED:
      return "[Armed] Triggered";
    case VISION_CALIBRATE:
      return "[Vision] Calibrate";
  }
  return "Unknown";
}

}  // namespace robot
