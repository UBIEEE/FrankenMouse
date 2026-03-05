#include <micromouse/vision/vision.hpp>
#include <micromouse/vision/vision_distances.hpp>

#define LOG_PREFIX "[vision] "
#include <micromouse/logging.hpp>

using namespace vision;

Vision::Vision() {
  //
}

void Vision::periodic() {
  m_was_left_wall = left_wall();
  m_was_right_wall = right_wall();
}

void Vision::reset_calibration() {
  LogInfo("reset vision calibration");
}

void Vision::calibrate() {
  LogInfo("calibrate vision");
}

bool Vision::left_wall() {
  return m_ir_sensors.get_distance(hardware::IRSensors::MID_LEFT) <
         (VisionDistances::NOMINAL_SIDE_WALL_DISTANCE + VisionDistances::DISTANCE_TOLERANCE);
}

bool Vision::right_wall() {
  return m_ir_sensors.get_distance(hardware::IRSensors::MID_RIGHT) <
         (VisionDistances::NOMINAL_SIDE_WALL_DISTANCE + VisionDistances::DISTANCE_TOLERANCE);
}

bool Vision::front_wall() {
  bool front_left = m_ir_sensors.get_distance(hardware::IRSensors::FAR_LEFT) <
                    (VisionDistances::NOMINAL_FRONT_WALL_DISTANCE + VisionDistances::DISTANCE_TOLERANCE);
  bool front_right = m_ir_sensors.get_distance(hardware::IRSensors::FAR_RIGHT) <
                     (VisionDistances::NOMINAL_FRONT_WALL_DISTANCE + VisionDistances::DISTANCE_TOLERANCE);

  return front_left && front_right;
}

bool Vision::did_left_wall_just_disappear() {
  return m_was_left_wall && !left_wall();
}

bool Vision::did_right_wall_just_disappear() {
  return m_was_right_wall && !right_wall();
}

void Vision::publish_periodic_feedback() {
  using namespace feedback;
  m_feedback.publish<TopicSend::VISION_RAW_READINGS>(m_ir_sensors.get_raw_readings());
  m_feedback.publish<TopicSend::VISION_DISTANCES>(m_ir_sensors.get_distances());
}
