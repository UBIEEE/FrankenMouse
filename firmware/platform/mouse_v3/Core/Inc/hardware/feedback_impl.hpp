#pragma once

#include <micromouse/hardware/feedback.hpp>

class FeedbackImpl : public hardware::Feedback {
 public:
  void publish_topic(feedback::TopicSend topic, const uint8_t* data) override;
};

FeedbackImpl& get_mouse_v3_feedback();
