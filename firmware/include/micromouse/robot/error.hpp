#pragma once

#include <cstdint>

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
};

// no padding
#pragma pack(push, 1)
struct Error {
  uint32_t timestamp;
  ErrorCategory category;
  uint8_t code;
};
#pragma pack(pop)

static_assert(sizeof(Error) == 6, "Error struct must be packed.");

}