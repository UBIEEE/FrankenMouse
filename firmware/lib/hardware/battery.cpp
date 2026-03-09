#include <micromouse/hardware/battery.hpp>

using namespace hardware;

class UnimplementedBattery : public Battery {
 public:
  float get_charge_percentage() override { return 1.f; }
  bool is_battery() const override { return true; }
};

__attribute__((weak)) Battery& get_platform_battery() {
  static UnimplementedBattery s_battery;
  return s_battery;
}
