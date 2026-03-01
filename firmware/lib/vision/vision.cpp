#include <micromouse/vision/vision.hpp>

#define LOG_PREFIX "[vision] "
#include <micromouse/logging.hpp>

using namespace vision;

Vision::Vision() {
  //
}

void Vision::periodic() {
  //
}

void Vision::reset_calibration() {
  LogInfo("reset vision calibration");
}

void Vision::calibrate() {
  LogInfo("calibrate vision");
}

bool Vision::left_wall() {
  return m_ir_sensors.get_distance(hardware::IRSensors::MID_LEFT) < 150_mm;
}

bool Vision::right_wall() {
  return m_ir_sensors.get_distance(hardware::IRSensors::MID_RIGHT) < 150_mm;
}

bool Vision::front_wall() {
  bool front_left =
      m_ir_sensors.get_distance(hardware::IRSensors::FAR_LEFT) < 180_mm;
  bool front_right =
      m_ir_sensors.get_distance(hardware::IRSensors::FAR_RIGHT) < 180_mm;

  return front_left || front_right;
}

void Vision::publish_periodic_feedback() {
  using namespace feedback;
  m_feedback.publish<TopicSend::VISION_RAW_READINGS>(m_ir_sensors.get_raw_readings());
  m_feedback.publish<TopicSend::VISION_DISTANCES>(m_ir_sensors.get_distances());
}
