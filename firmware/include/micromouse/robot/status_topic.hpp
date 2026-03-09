#pragma once

#include <cstdint>

namespace robot {

enum class StatusTopic : uint8_t {
  IS_VISION_CALIBRATED = 0,
  POWER_SOURCE = 1,
  BATTERY_STATUS = 2,
};

}  // namespace robot
