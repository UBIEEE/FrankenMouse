#pragma once

#include <micromouse/feedback_topic.hpp>
#include <micromouse/hardware/component.hpp>

namespace hardware {

class Feedback : public Component {
 protected:
  Feedback() = default;

 public:
  virtual void publish_topic(FeedbackTopicSend topic, uint8_t* data) = 0;
};

}  // namespace hardware

/**
 * @brief Returns an instance of the platform-specific feedback.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return hardware::Feedback&
 */
hardware::Feedback& get_platform_feedback();
