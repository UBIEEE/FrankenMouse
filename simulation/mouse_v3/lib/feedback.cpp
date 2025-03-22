#include <godot_cpp/core/class_db.hpp>
#include <simulator/feedback.hpp>

#include <cassert>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace godot;

class FeedbackNode : public rclcpp::Node {
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      m_ir_readings_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_button1_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_button2_pub;

  MicroMouse* m_mouse;

  bool m_waving = false;

 public:
  FeedbackNode(MicroMouse* mouse) : rclcpp::Node("simulation"), m_mouse(mouse) {
    m_ir_readings_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/simulation/vision/ir_readings", 10);

    m_cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
        "/simulation/drive/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist& msg) {
          m_mouse->set_linear_velocity(msg.linear.x);
          m_mouse->set_angular_velocity(msg.angular.z);
        });

    m_button1_pub = create_publisher<std_msgs::msg::Bool>(
        "/simulation/buttons/button1", 10);
    m_button2_pub = create_publisher<std_msgs::msg::Bool>(
        "/simulation/buttons/button2", 10);
  }

  void process() {
    std_msgs::msg::Float32MultiArray ir_data;

    if (m_waving) {
      ir_data.data = std::vector<float>(8, 1.f);
    } else {
      ir_data.data.resize(8);

      const float* raw_readings = m_mouse->get_ir_sensor_readings();
      const float* distances = m_mouse->get_ir_sensor_distances_mm();
      std::copy(raw_readings, raw_readings + 4, ir_data.data.begin());
      std::copy(distances, distances + 4, ir_data.data.begin() + 4);
    }

    m_ir_readings_pub->publish(ir_data);
  }

  void set_waving(bool waving) { m_waving = waving; }

  void button_1(bool pressed) {
    std_msgs::msg::Bool msg;
    msg.data = pressed;
    m_button1_pub->publish(msg);
  }

  void button_2(bool pressed) {
    std_msgs::msg::Bool msg;
    msg.data = pressed;
    m_button2_pub->publish(msg);
  }
};

void Feedback::_bind_methods() {}

Feedback::Feedback() {}

Feedback::~Feedback() {
  if (m_init)
    rclcpp::shutdown();
}

void Feedback::_ready() {
  godot::Node* parent = get_parent();

  MicroMouse* mouse = parent->get_node<MicroMouse>("MicroMouse");
  assert(mouse);

  rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  m_ros2_node = std::make_shared<FeedbackNode>(mouse);

  m_btn1_button = parent->get_node<Button>("BTN1Button");
  m_btn2_button = parent->get_node<Button>("BTN2Button");
  m_wave_button = parent->get_node<Button>("WaveButton");
  assert(m_btn1_button);
  assert(m_btn2_button);
  assert(m_wave_button);

  m_btn1_button->connect("button_down",
                         callable_mp(this, &Feedback::button_1_down));
  m_btn1_button->connect("button_up",
                         callable_mp(this, &Feedback::button_1_up));
  m_btn2_button->connect("button_down",
                         callable_mp(this, &Feedback::button_2_down));
  m_btn2_button->connect("button_up",
                         callable_mp(this, &Feedback::button_2_up));
  m_wave_button->connect("button_down",
                         callable_mp(this, &Feedback::wave_down));
  m_wave_button->connect("button_up", callable_mp(this, &Feedback::wave_up));

  m_init = true;
}

void Feedback::_process(double delta) {
  if (!m_init)
    return;

  m_ros2_node->process();
  rclcpp::spin_some(m_ros2_node);
}

void Feedback::button_1_down() {
  m_ros2_node->button_1(true);
}

void Feedback::button_1_up() {
  m_ros2_node->button_1(false);
}

void Feedback::button_2_down() {
  m_ros2_node->button_2(true);
}

void Feedback::button_2_up() {
  m_ros2_node->button_2(false);
}

void Feedback::wave_down() {
  m_ros2_node->set_waving(true);
}

void Feedback::wave_up() {
  m_ros2_node->set_waving(false);
}

