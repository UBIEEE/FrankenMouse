#pragma once

#include <cstdint>

enum class RobotCommand : uint8_t {
  RESEND_ALL_FEEDBACK = 0,
  RESET_MAZE = 1,

  _COUNT,
};
