#include <micromouse/robot/error.hpp>

using namespace robot;

#ifdef WITH_LOGGING

#include <format>

std::string Error::to_string() const {
  std::string category_string = "UNKNOWN";
  ErrorCategory c = static_cast<ErrorCategory>(category);
  switch (c) {
    using enum ErrorCategory;
    case INVALID:
      category_string = "INVALID";
      break;
    case GENERAL:
      category_string = "GENERAL";
      break;
    case DRIVE:
      category_string = "DRIVE";
      break;
    case VISION:
      category_string = "VISION";
      break;
    case NAVIGATION:
      category_string = "NAVIGATION";
      break;
  }

  std::string code_string = "UNKNOWN";
  if (c == ErrorCategory::GENERAL) {
    GeneralErrorCode code = static_cast<GeneralErrorCode>(this->code);
    switch (code) {
      using enum GeneralErrorCode;
      case UNKNOWN:
        code_string = "UNKNOWN";
        break;
      case LOW_BATTERY:
        code_string = "LOW_BATTERY";
        break;
      case CANT_SOLVE_HAVENT_SEARCHED_YET:
        code_string = "CANT_SOLVE_HAVENT_SEARCHED_YET";
        break;
    }
  } else if (c == ErrorCategory::DRIVE) {
    DriveErrorCode code = static_cast<DriveErrorCode>(this->code);
    switch (code) {
      using enum DriveErrorCode;
      case UNKNOWN:
        code_string = "UNKNOWN";
        break;
    }
  } else if (c == ErrorCategory::VISION) {
    VisionErrorCode code = static_cast<VisionErrorCode>(this->code);
    switch (code) {
      using enum VisionErrorCode;
      case UNKNOWN:
        code_string = "UNKNOWN";
        break;
    }
  } else if (c == ErrorCategory::NAVIGATION) {
    NavigationErrorCode code = static_cast<NavigationErrorCode>(this->code);
    switch (code) {
      using enum NavigationErrorCode;
      case UNKNOWN:
        code_string = "UNKNOWN";
        break;
      case MAZE_UNSOLVABLE:
        code_string = "MAZE_UNSOLVABLE";
        break;
      case MAZE_WALL_INCONSISTENCY:
        code_string = "MAZE_WALL_INCONSISTENCY";
        break;
      case MAZE_EXIT_IN_BOUNDARY:
        code_string = "MAZE_EXIT_IN_BOUNDARY";
        break;
    }
  }

  return std::format("(timestamp: {}, category: {}, code: {})", timestamp, category_string, code_string);
}

#endif
