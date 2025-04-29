#include <micromouse/vision/vision.hpp>

using namespace vision;

Vision::Vision() {
  //
}

void Vision::periodic() {
  //
}

void Vision::reset_calibration() {
  //
}

void Vision::calibrate() {
  //
}

bool Vision::left_wall() {
  return m_ir_sensors.get_distance_mm(hardware::IRSensors::MID_LEFT) < 150.f;
}

bool Vision::right_wall() {
  return m_ir_sensors.get_distance_mm(hardware::IRSensors::MID_RIGHT) < 150.f;
}

bool Vision::front_wall() {
  bool front_left =
      m_ir_sensors.get_distance_mm(hardware::IRSensors::FAR_LEFT) < 180.f;
  bool front_right =
      m_ir_sensors.get_distance_mm(hardware::IRSensors::FAR_RIGHT) < 180.f;

  return front_left || front_right;
}

void Vision::publish_periodic_feedback() {
  m_feedback.publish_topic(FeedbackTopicSend::VISION_RAW_READINGS,
                           (uint8_t*)m_ir_sensors.get_raw_readings());
  m_feedback.publish_topic(FeedbackTopicSend::VISION_DISTANCES,
                           (uint8_t*)m_ir_sensors.get_distances_mm());
}
