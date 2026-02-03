#pragma once

#include <cstdint>

enum class RobotErrorCategory : uint8_t {
  INVALID = 0,
  GENERAL,
  DRIVE,
  VISION,
  NAVIGATION,
};

enum class RobotGeneralErrorCode : uint8_t {
  UNKNOWN = 0,
  LOW_BATTERY,
};

enum class RobotDriveErrorCode : uint8_t {
  UNKNOWN = 0,
};

enum class RobotVisionErrorCode : uint8_t {
  UNKNOWN = 0,
};

enum class RobotNavigationErrorCode : uint8_t {
  UNKNOWN = 0,
  MAZE_UNSOLVABLE,
  MAZE_WALL_INCONSISTENCY,
};

// no padding
#pragma pack(push, 1)
struct RobotError {
  uint32_t timestamp;
  RobotErrorCategory category;
  uint8_t code;
};
#pragma pack(pop)

static_assert(sizeof(RobotError) == 6, "RobotError struct must be packed.");
