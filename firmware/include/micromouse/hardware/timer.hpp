#pragma once

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

  virtual uint32_t elapsed_ms() const = 0;
  float elapsed_s() const { return elapsed_ms() / 1000.0f; }
};

}  // namespace hardware

/**
 * @brief Returns a newly created platform-specific timer.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return std::unique_ptr<hardware::Timer>
 */
std::unique_ptr<hardware::Timer> make_platform_timer();
