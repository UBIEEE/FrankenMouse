#include <micromouse/feedback/feedback_topic.hpp>

namespace feedback {

const char* topic_receive_to_string(TopicReceive topic) {
  switch (topic) {
    using enum TopicReceive;
    case MAIN_TASK:
      return "[Main] Task";
    case MAIN_COMMAND:
      return "[Main] Command";
    case MAIN_SONG:
      return "[Main] Song";
    case DRIVE_PID:
      return "[Drive] PID";
    case DRIVE_CHASSIS_SPEEDS:
      return "[Drive] Chassis Speeds";
  }
  return "[Unknown]";
}

const char* topic_send_to_string(TopicSend topic) {
  switch (topic) {
    using enum TopicSend;
    case MAIN_TASK:
      return "[Main] Task";
    case MAIN_ERROR:
      return "[Main] Error";
    case MAIN_SONG:
      return "[Main] Song";
    case MAIN_STATUS:
      return "[Main] Status";
    case VISION_RAW_READINGS:
      return "[Vision] Raw Readings";
    case VISION_DISTANCES:
      return "[Vision] Distances";
    case DRIVE_MOTOR_DATA:
      return "[Drive] Motor Data";
    case DRIVE_IMU_DATA:
      return "[Drive] IMU Data";
    case DRIVE_PID:
      return "[Drive] PID";
    case DRIVE_CHASSIS_SPEEDS:
      return "[Drive] Chassis Speeds";
    case MAZE_CELL:
      return "[Maze] Cell";
    case MAZE_MOUSE_POSITION:
      return "[Maze] Mouse Position";
  }
  return "[Unknown]";
}

}  // namespace feedback