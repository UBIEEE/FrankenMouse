#include <micromouse/hardware/drivetrain.hpp>

using namespace hardware;

class UnimplementedDrivetrain : public Drivetrain {
 public:
  void stop() override {}
  void set_wheel_speeds(const drive::WheelSpeeds&) override {}
  void set_chassis_speeds(const drive::ChassisSpeeds&) override {}
};

__attribute__((weak)) Drivetrain& get_platform_drivetrain() {
  static UnimplementedDrivetrain s_drivetrain;
  return s_drivetrain;
}
