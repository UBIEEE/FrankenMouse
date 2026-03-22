#pragma once

#include <cstdint>
#include <concepts>
#include <string>

namespace robot {

enum class ErrorCategory : uint8_t {
  INVALID = 0,
  GENERAL,
  DRIVE,
  VISION,
  NAVIGATION,
};

enum class GeneralErrorCode : uint8_t {
  UNKNOWN = 0,
  LOW_BATTERY,
};

enum class DriveErrorCode : uint8_t {
  UNKNOWN = 0,
};

enum class VisionErrorCode : uint8_t {
  UNKNOWN = 0,
};

enum class NavigationErrorCode : uint8_t {
  UNKNOWN = 0,
  MAZE_UNSOLVABLE,
  MAZE_WALL_INCONSISTENCY,
  MAZE_EXIT_IN_BOUNDARY,
};

template <typename T>
concept ErrorCode = std::same_as<T, GeneralErrorCode> || std::same_as<T, DriveErrorCode> ||
                    std::same_as<T, VisionErrorCode> || std::same_as<T, NavigationErrorCode>;

// no padding
#pragma pack(push, 1)
struct Error {
  uint32_t timestamp;
  ErrorCategory category;
  uint8_t code;

  template <ErrorCode Code>
  static constexpr Error create(uint32_t timestamp, Code code) {
    Error e;
    e.timestamp = timestamp;
    e.code = static_cast<uint8_t>(code);
    if constexpr (std::same_as<Code, GeneralErrorCode>) {
      e.category = ErrorCategory::GENERAL;
    } else if constexpr (std::same_as<Code, DriveErrorCode>) {
      e.category = ErrorCategory::DRIVE;
    } else if constexpr (std::same_as<Code, VisionErrorCode>) {
      e.category = ErrorCategory::VISION;
    } else if constexpr (std::same_as<Code, NavigationErrorCode>) {
      e.category = ErrorCategory::NAVIGATION;
    } else {
      e.category = ErrorCategory::INVALID;
    }
    return e;
  }

#ifdef WITH_LOGGING
  std::string to_string() const;
#endif
};
#pragma pack(pop)

static_assert(sizeof(Error) == 6, "Error struct must be packed.");

}  // namespace robot
