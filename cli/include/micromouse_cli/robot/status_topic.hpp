#pragma once

#include <cstdint>

enum class RobotStatusTopic : uint8_t {
  IS_VISION_CALIBRATED = 0,
  POWER_SOURCE = 1,
  BATTERY_STATUS = 2,
};
