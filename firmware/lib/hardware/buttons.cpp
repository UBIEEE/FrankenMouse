#include <micromouse/hardware/buttons.hpp>

using namespace hardware;

class UnimplementedButtons : public Buttons {
 public:
  void register_button_1_callback(std::function<void()>) override {}
  void register_button_2_callback(std::function<void()>) override {}
};

__attribute__((weak)) hardware::Buttons& get_platform_buttons() {
  static UnimplementedButtons buttons;
  return buttons;
}
