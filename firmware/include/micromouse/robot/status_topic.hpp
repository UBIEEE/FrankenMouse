#pragma once

#include <cstdint>

namespace robot {

enum class StatusTopic : uint8_t {
  NONE = 0,

  MAZE_WALL_GONE,
};

#pragma pack(push, 1)
struct StatusUpdate {
  StatusTopic topic = StatusTopic::NONE;
  float value = 0;
};
#pragma pack(pop)
static_assert(sizeof(StatusUpdate) == 5);

}  // namespace robot
