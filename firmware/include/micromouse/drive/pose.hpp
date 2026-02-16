#pragma once

#include <units/length.h>
#include <units/angle.h>

namespace drive {

struct Pose {
  units::millimeter_t x = 0_mm;
  units::millimeter_t y = 0_mm;
  units::degree_t theta = 0_deg;
};

}  // namespace drive
