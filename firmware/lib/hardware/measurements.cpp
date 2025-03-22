#include <micromouse/hardware/measurements.hpp>

using namespace hardware;

__attribute__((weak)) RobotMeasurements& get_robot_measurements() {
  static RobotMeasurements defaultMeasurements;
  return defaultMeasurements;
}
