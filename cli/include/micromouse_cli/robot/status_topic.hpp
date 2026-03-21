#pragma once

#include <cstdint>

enum class RobotStatusTopic : uint8_t {
  NONE = 0,
};

#pragma pack(push, 1)
struct RobotStatusUpdate {
  RobotStatusTopic topic = RobotStatusTopic::NONE;
  float value = 0;
};
#pragma pack(pop)
static_assert(sizeof(RobotStatusUpdate) == 5);
