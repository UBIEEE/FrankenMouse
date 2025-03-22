#include "hardware/buzzer_impl.hpp"

#include "stm32wbxx_hal.h"

extern TIM_HandleTypeDef htim16;  // main.c

using namespace audio;

static uint32_t s_aar_octave_4[] = {
    0,     // NONE
    0,     // C
    0,     // D
    0,     // E
    0,     // F
    3360,  // G
    2995,  // A
    2690,  // B
    0,     // CS
    0,     // DS
    0,     // FS
    3180,  // GS
    2840,  // AS
};

static uint32_t s_aar_octave_5[] = {
    0,     // NONE
    2545,  // C
    2280,  // D
    2040,  // E
    1930,  // F
    1740,  // G
    720,   // A
    640,   // B
    2410,  // CS
    2160,  // DS
    1830,  // FS
    760,   // GS
    680,   // AS
};

static uint32_t s_aar_octave_6[] = {
    0,    // NONE
    0,    // C
    540,  // D
    480,  // E
    0,    // F
    0,    // G
    0,    // A
    0,    // B
    570,  // CS
    0,    // DS
    0,    // FS
    0,    // GS
    0,    // AS
};

void BuzzerImpl::play_note(audio::Note note) {
  uint32_t aar = 0;

  switch (note.octave) {
    case 4:
      aar = s_aar_octave_4[int(note.note)];
      break;
    case 5:
      aar = s_aar_octave_5[int(note.note)];
      break;
    case 6:
      aar = s_aar_octave_6[int(note.note)];
      break;
  }

  htim16.Instance->ARR = aar;

  // 50% duty cycle for a square wave.
  htim16.Instance->CCR1 = aar / 2;
}

BuzzerImpl& get_mouse_v2_buzzer() {
  static BuzzerImpl s_buzzer;
  return s_buzzer;
}

hardware::Buzzer& get_platform_buzzer() {
  return get_mouse_v2_buzzer();
}
