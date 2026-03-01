#pragma once

#include <cstdint>

namespace robot {

enum class Command : uint8_t {
  RESEND_ALL_FEEDBACK = 0,
  RESET_MAZE,
  CALIBRATE_VISION,
  RESET_VISION_CALIBRATION,
};

}  // namespace robot
