#include <micromouse/hardware/buttons.hpp>

using namespace hardware;

class UnimplementedButtons : public Buttons {
 public:
  void register_btn1_callback(std::function<void()>) {}
  void register_btn2_callback(std::function<void()>) {}
};

__attribute__((weak)) hardware::Buttons& get_platform_buttons() {
  static UnimplementedButtons buttons;
  return buttons;
}
