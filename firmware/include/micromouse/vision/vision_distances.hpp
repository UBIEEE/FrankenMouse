#pragma once

#include <units/length.h>

namespace vision {

class VisionDistances {
 public:
  static constexpr units::millimeter_t NOMINAL_SIDE_WALL_DISTANCE = 86_mm;
  static constexpr units::millimeter_t NOMINAL_FRONT_WALL_DISTANCE = 156_mm;

  static constexpr units::millimeter_t DISTANCE_TOLERANCE = 65_mm;
};

}  // namespace vision
