#pragma once

namespace drive {

struct ChassisSpeeds {
  float linear_velocity_mmps;  // positive is forward
  float angular_velocity_dps;  // positive is CCW
};

}  // namespace drive
