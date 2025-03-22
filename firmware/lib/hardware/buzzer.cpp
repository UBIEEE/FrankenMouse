#include <micromouse/hardware/buzzer.hpp>

using namespace hardware;

class UnimplementedBuzzer : public Buzzer {
 public:
  void play_note(audio::Note) override {}
};

__attribute__((weak)) Buzzer& get_platform_buzzer() {
  static UnimplementedBuzzer s_buzzer;
  return s_buzzer;
}
