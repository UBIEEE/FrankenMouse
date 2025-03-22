#include <micromouse/hardware/feedback.hpp>

using namespace hardware;

class UnimplementedFeedback : public Feedback {
 public:
  void publish_topic(FeedbackTopicSend, uint8_t*) override {}
};

__attribute__((weak)) Feedback& get_platform_feedback() {
  static UnimplementedFeedback s_feedback;
  return s_feedback;
}
