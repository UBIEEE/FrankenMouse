#include <micromouse/hardware/battery.hpp>

using namespace hardware;

class UnimplementedBattery : public Battery {
 public:
  float get_charge_percentage() const override { return 1.f; }
  units::volt_t get_voltage() const override { return 8_V; }
  bool is_battery() const override { return true; }
};

__attribute__((weak)) Battery& get_platform_battery() {
  static UnimplementedBattery s_battery;
  return s_battery;
}
