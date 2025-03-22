#include <micromouse/hardware/ir_sensors.hpp>

#include <algorithm>
#include <limits>

using namespace hardware;

class UnimplementedIRSensors : public IRSensors {
  float m_readings[4] = {0};
  float m_distances_mm[4];

 public:
  UnimplementedIRSensors() {
    std::fill(m_distances_mm, m_distances_mm + 4,
              std::numeric_limits<float>::infinity());
  }

  const float* get_raw_readings() const override { return m_readings; };
  const float* get_distances_mm() const override { return m_distances_mm; };
};

__attribute__((weak)) IRSensors& get_platform_ir_sensors() {
  static UnimplementedIRSensors sensors;
  return sensors;
}
