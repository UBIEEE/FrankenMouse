#include <micromouse/robot/command.hpp>

namespace robot {

const char* command_to_string(Command command) {
  switch (command) {
    using enum Command;
    case RESEND_ALL_FEEDBACK:
      return "Resend All Feedback";
    case RESET_MAZE:
      return "Reset Maze";
    case CALIBRATE_VISION:
      return "Calibrate Vision";
    case RESET_VISION_CALIBRATION:
      return "Reset Vision Calibration";
  }
  return "Unknown";
}

}  // namespace robot
