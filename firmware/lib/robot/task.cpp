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
    case MAZE_SEARCH_TWO_TIMES:
      return "[Maze] Search (Two Times)";
    case MAZE_SEARCH_TWO_TIMES_START_STOP_MOTION:
      return "[Maze] Search (Two Times) (Start+Stop Motion)";
    case MAZE_SOLVE_WITH_SEARCH_NAVIGATION:
      return "[Maze] Solve with Search Navigation";
    case MAZE_SOLVE_WITH_SEARCH_NAVIGATION_START_STOP_MOTION:
      return "[Maze] Solve with Search Navigation (Start+Stop Motion)";
    case MAZE_SEARCH_FASTER:
      return "[Maze] Search (Faster)";
    case MAZE_SEARCH_THEN_SLOW_SOLVE_THEN_FAST_SOLVE:
      return "[Maze] Search then slow solve then fast solve";
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
    case DRIVE_BACKUP_INTO_WALL:
      return "[Drive] Backup into wall";
    case TEST_GYRO:
      return "[Test] Gyro";
    case TEST_DRIVE_STRAIGHT_FOUR_CELLS_FROM_BACK_WALL_WITH_VISION_ALIGN:
      return "[Test] Drive Straight Four Cells from Back Wall with Vision Align";
    case TEST_DRIVE_RAW_SPEEDS:
      return "[Test] Drive Raw Speeds";
    case TEST_DRIVE_CONSTANT_SPEED:
      return "[Test] Drive Constant Speed";
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
