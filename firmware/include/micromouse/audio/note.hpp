#pragma once

#include <cstdint>

namespace audio {

enum class NoteInOctave {
  NONE,

  C,
  D,
  E,
  F,
  G,
  A,
  B,

  C_SHARP,
  D_SHARP,
  E_SHARP = F,
  F_SHARP,
  G_SHARP,
  A_SHARP,
  B_SHARP = C,

  C_FLAT = B,
  D_FLAT = C_SHARP,
  E_FLAT = D_SHARP,
  F_FLAT = E,
  G_FLAT = F_SHARP,
  A_FLAT = G_SHARP,
  B_FLAT = A_SHARP,

  CS = C_SHARP,
  DS = D_SHARP,
  ES = E_SHARP,
  FS = F_SHARP,
  GS = G_SHARP,
  AS = A_SHARP,
  BS = B_SHARP,

  CF = C_FLAT,
  DF = D_FLAT,
  EF = E_FLAT,
  FF = F_FLAT,
  GF = G_FLAT,
  AF = A_FLAT,
  BF = B_FLAT,
};

struct Note {
  NoteInOctave note;
  uint8_t octave;
};

constexpr const Note REST = {NoteInOctave::NONE, 0};

}  // namespace audio
