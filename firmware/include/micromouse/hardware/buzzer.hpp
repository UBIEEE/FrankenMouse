#pragma once

#include <micromouse/audio/note.hpp>
#include <micromouse/hardware/component.hpp>

namespace hardware {

class Buzzer : public Component {
 protected:
  Buzzer() = default;

 public:
  virtual void play_note(audio::Note note) = 0;
};

}  // namespace hardware

/**
 * @brief Returns an instance of the platform-specific buzzer.
 *
 * This function is to be implemented by the user in platform-specific code.
 *
 * @return hardware::Buzzer&
 */
hardware::Buzzer& get_platform_buzzer();
