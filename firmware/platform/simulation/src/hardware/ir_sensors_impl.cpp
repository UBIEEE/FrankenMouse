#include <simulation/hardware/ir_sensors_impl.hpp>

#include <limits>
#include <algorithm>

using namespace std::placeholders;

IRSensorsImpl::IRSensorsImpl() : Node("micromouse_ir_sensors") {
  std::fill(m_distances_mm, m_distances_mm + 4,
            std::numeric_limits<float>::infinity());

  m_ir_readings_sub =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/simulation/vision/ir_readings", 10,
          std::bind(&IRSensorsImpl::ir_readings_callback, this, _1));
}

void IRSensorsImpl::ir_readings_callback(
    const std_msgs::msg::Float32MultiArray& msg) {
  assert(msg.data.size() == 8);

  std::copy(msg.data.begin(), msg.data.begin() + 4, m_raw_readings);
  std::copy(msg.data.begin() + 4, msg.data.end(), m_distances_mm);
}

std::shared_ptr<IRSensorsImpl> get_simulation_ir_sensors() {
  static auto ir_sensors = std::make_shared<IRSensorsImpl>();
  return ir_sensors;
}

hardware::IRSensors& get_platform_ir_sensors() {
  return *get_simulation_ir_sensors();
}
