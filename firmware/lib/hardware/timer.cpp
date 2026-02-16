#include <micromouse/hardware/timer.hpp>

using namespace hardware;

class UnimplementedTimer : public Timer {
 public:
  void reset() override {}
  void start() override {}
  void stop() override {}
  units::millisecond_t get() const override { return 0_ms; }
};

__attribute__((weak)) std::unique_ptr<Timer> make_platform_timer() {
  return std::make_unique<UnimplementedTimer>();
}
