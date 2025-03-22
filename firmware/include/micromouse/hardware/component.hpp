#pragma once

namespace hardware {

class Component {
 public:
  virtual ~Component() = default;

  Component(const Component&) = delete;
  Component(Component&&) = delete;
  Component& operator=(const Component&) = delete;
  Component& operator=(Component&&) = delete;

  virtual void periodic() {}
  virtual void publish_periodic_feedback() {}
  virtual void publish_extra_feedback() {}

 protected:
  Component() = default;
};

}  // namespace hardware
