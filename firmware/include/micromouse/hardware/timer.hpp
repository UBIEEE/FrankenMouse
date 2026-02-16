#pragma once

#include <units/time.h>
#include <cstdint>
#include <memory>

namespace hardware {

class Timer {
 protected:
  Timer() = default;

 public:
  virtual ~Timer() = default;

  virtual void reset() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;

  virtual units::millisecond_t get() const = 0;
};

}  // namespace hardware

/**
 * Returns a newly created platform-specific timer.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return The timer instance.
 */
std::unique_ptr<hardware::Timer> make_platform_timer();
