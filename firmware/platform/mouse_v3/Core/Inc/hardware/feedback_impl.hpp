#pragma once

#include <micromouse/hardware/feedback.hpp>

class FeedbackImpl : public hardware::Feedback {
 public:
  void publish_topic(FeedbackTopicSend topic, uint8_t* data) override;
};

FeedbackImpl& get_mouse_v2_feedback();
