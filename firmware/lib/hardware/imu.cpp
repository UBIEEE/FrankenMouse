#include <micromouse/hardware/imu.hpp>

using namespace hardware;

class UnimplementedIMU : public IMU {
 public:
  units::degrees_per_second_t get_angular_velocity(Axis) override { return 0_deg_per_s; }
  units::standard_gravity_t get_linear_accel(Axis) override { return 0_SG; }
};

__attribute__((weak)) IMU& get_platform_imu() {
  static UnimplementedIMU s_imu;
  return s_imu;
}
