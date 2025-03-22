#pragma once

class Subsystem {
 public:
  virtual ~Subsystem() = default;

  Subsystem(const Subsystem&) = delete;
  Subsystem(Subsystem&&) = delete;
  Subsystem& operator=(const Subsystem&) = delete;
  Subsystem& operator=(Subsystem&&) = delete;

  virtual void periodic() {}
  virtual void publish_periodic_feedback() {}
  virtual void publish_extra_feedback() {}

 protected:
  Subsystem() = default;
};
