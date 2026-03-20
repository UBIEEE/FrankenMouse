#include <micromouse/hardware/timer.hpp>
#include "units/time.h"

using namespace hardware;

class UnimplementedTimer : public Timer {
 public:
  void reset() override {}
  void start() override {}
  void stop() override {}
  units::millisecond_t get() const override { return 0_ms; }
};

__attribute__((weak)) units::millisecond_t get_system_timestamp() {
  return 0_ms;
}

__attribute__((weak)) std::unique_ptr<Timer> make_platform_timer() {
  return std::make_unique<UnimplementedTimer>();
}
