#pragma once

#include <micromouse/feedback/feedback_topic.hpp>
#include <micromouse/hardware/component.hpp>

namespace hardware {

class Feedback : public Component {
 protected:
  Feedback() = default;

  virtual void publish_topic(feedback::TopicSend topic, const uint8_t* data) = 0;

 public:
  template <feedback::TopicSend Topic>
  void publish(const typename feedback::TopicSendData<Topic>::type& value) {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&value);
    publish_topic(Topic, data);  //, sizeof(ValueType));
  }
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
