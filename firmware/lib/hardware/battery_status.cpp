#include <micromouse/hardware/battery_status.hpp>

using namespace hardware;

class UnimplementedBatteryStatus : public BatteryStatus {
 public:
  float get_charge_percentage() override { return 1.f; }
  bool is_battery() const override { return true; }
};

__attribute__((weak)) BatteryStatus& get_platform_battery_status() {
  static UnimplementedBatteryStatus s_battery_status;
  return s_battery_status;
}
