#include <micromouse/hardware/imu.hpp>

using namespace hardware;

class UnimplementedIMU : public IMU {
 public:
  float get_angular_velocity(Axis) override { return 0.f; }
  float get_linear_accel(Axis) override { return 0.f; }
};

__attribute__((weak)) IMU& get_platform_imu() {
  static UnimplementedIMU s_imu;
  return s_imu;
}
