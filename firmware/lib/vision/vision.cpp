#include <micromouse/vision/vision.hpp>

using namespace vision;

Vision::Vision() {
  //
}

void Vision::periodic() {
  //
}

void Vision::publish_periodic_feedback() {
  m_feedback.publish_topic(FeedbackTopicSend::VISION_RAW_READINGS, (uint8_t*)m_ir_sensors.get_raw_readings());
  m_feedback.publish_topic(FeedbackTopicSend::VISION_DISTANCES, (uint8_t*)m_ir_sensors.get_distances_mm());
}

void Vision::reset_calibration() {
  //
}

void Vision::calibrate() {
  //
}
