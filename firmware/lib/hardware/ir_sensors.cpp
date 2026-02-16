#include <micromouse/hardware/ir_sensors.hpp>

#include <algorithm>
#include <limits>

using namespace hardware;

class UnimplementedIRSensors : public IRSensors {
  std::array<float, 4> m_raw_readings = {0};
  std::array<units::millimeter_t, 4> m_distances;

 public:
  UnimplementedIRSensors() {
  std::fill(m_distances.begin(), m_distances.end(), units::millimeter_t{std::numeric_limits<float>::infinity()});
  }

  const std::array<float, 4>& get_raw_readings() const override { return m_raw_readings; }
  const std::array<units::millimeter_t, 4>& get_distances() const override { return m_distances; }
};

__attribute__((weak)) IRSensors& get_platform_ir_sensors() {
  static UnimplementedIRSensors sensors;
  return sensors;
}
