#pragma once

#include <godot_cpp/classes/button.hpp>
#include <godot_cpp/classes/node.hpp>

#include <simulator/micromouse.hpp>

class FeedbackNode;

namespace godot {

class Feedback : public Node {
  GDCLASS(Feedback, Node)

 private:
  bool m_init = false;

  std::shared_ptr<FeedbackNode> m_ros2_node;

  Button* m_btn1_button;
  Button* m_btn2_button;
  Button* m_wave_button;

 protected:
  static void _bind_methods();

 public:
  Feedback();
  ~Feedback();

  void _ready() override;
  void _process(double delta) override;

 private:
  void button_1_down();
  void button_1_up();
  void button_2_down();
  void button_2_up();

  void wave_down();
  void wave_up();
};

}  // namespace godot
