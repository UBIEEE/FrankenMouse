#pragma once

#include <micromouse/hardware/buzzer.hpp>

class BuzzerImpl : public hardware::Buzzer {
 public:
  void play_note(audio::Note note) override;
};

BuzzerImpl& get_mouse_v2_buzzer();
