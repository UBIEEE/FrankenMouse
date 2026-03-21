#include <micromouse/audio/audio_player.hpp>
#include "micromouse/audio/note.hpp"
#include "micromouse/audio/song.hpp"

#include <micromouse/robot/robot.h>

namespace audio {

using enum NoteInOctave;

const char* song_to_string(Song song) {
  switch (song) {
    using enum Song;
    case QUIET:
      return "Quiet";
    case HOME_DEPOT:
      return "Home Depot Theme";
    case NOKIA:
      return "Nokia Ringtone";
    case WINDOWS_XP_SHUTDOWN:
      return "Windows XP Shutdown";
    case STARTUP:
      return "Startup Tone";
    case BLE_CONNECT:
      return "BLE Connect Tone";
    case BLE_DISCONNECT:
      return "BLE Disconnect Tone";
    case BEGIN_SEARCH:
      return "Begin Search Tone";
    case BEGIN_FAST_SOLVE:
      return "Begin Fast Solve Tone";
    case BEGIN_SLOW_SOLVE:
      return "Begin Slow Solve Tone";
    case BEGIN_OTHER:
      return "Begin Other Tone";
    case ARMED:
      return "Armed Tone";
    case ARMED_TRIGGERING:
      return "Armed and Triggering Tone";
  }
  return "Unknown";
}

// clang-format off

// Startup tone.

static constexpr uint16_t SONG_STARTUP_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_STARTUP_NOTES[] = {
    {D, 5},
    {D, 6},
};

// BLE connect tone.

static constexpr uint16_t SONG_BLE_CONNECT_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_BLE_CONNECT_NOTES[] = {
    {E, 5},
    {G, 5},
    {E, 6},
};

// BLE disconnect tone.

static constexpr uint16_t SONG_BLE_DISCONNECT_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_BLE_DISCONNECT_NOTES[] = {
    {E, 5},
    {G, 5},
    {G, 4},
};

// Home Depot theme song.

static constexpr uint16_t SONG_HOME_DEPOT_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_HOME_DEPOT_NOTES[] = {
    // Intro
    {A, 4}, {A, 4}, {D, 5}, {A, 4}, REST, {A, 4}, REST, {A, 4}, {C, 5}, {A, 4}, REST, {A, 4}, REST, {A, 4}, {G, 4}, {A, 4},
    {A, 4}, {A, 4}, {D, 5}, {A, 4}, REST, {A, 4}, REST, {A, 4}, {C, 5}, {A, 4}, REST, {A, 4}, REST, {A, 4}, {G, 4}, {A, 4},
    {A, 4}, {A, 4}, {D, 5}, {A, 4}, REST, {A, 4}, REST, {A, 4}, {C, 5}, {A, 4}, REST, {A, 4}, REST, {A, 4}, {G, 4}, {A, 4},

    // Transition
    REST, {A, 4}, {D, 5}, {A, 4}, {C, 5}, {D, 6}, REST,

    // Loop
    {A, 4}, {D, 5}, {A, 4}, {C, 5}, {A, 4}, {G, 4}, {D, 6}, REST,
    {A, 4}, {D, 5}, {A, 4}, {C, 5}, {A, 4}, {G, 4}, {D, 6}, REST,
    {A, 4}, {D, 5}, {A, 4}, {C, 5}, {A, 4}, {G, 4}, {D, 6}, REST,
    {A, 4}, {D, 5}, {A, 4}, {C, 5}, {A, 4}, {G, 4}, {D, 6}, REST,
    {A, 4}, {D, 5}, {A, 4}, {C, 5}, {A, 4}, {G, 4}, {D, 6}, REST,
    {A, 4}, {D, 5}, {A, 4}, {C, 5}, {A, 4}, {G, 4}, {D, 6}, REST,
    {A, 4}, {D, 5}, {A, 4}, {C, 5}, {A, 4}, {G, 4}, {D, 6}, REST,

    // End
    {A, 4}, {D, 5}, {A, 4}, {A, 4},
};

// Nokia ringtone.

static constexpr uint16_t SONG_NOKIA_NOTE_LENGTH_MS = 180;

static constexpr Note SONG_NOKIA_NOTES[] = {
    {E,  6}, {D, 6}, {FS, 5}, {FS, 5}, {GS, 5}, {GS, 5},
    {CS, 6}, {B, 5}, {D,  5}, {D,  5}, {E,  5}, {E,  5},
    {B,  5}, {A, 5}, {CS, 5}, {CS, 5}, {E,  5}, {E,  5},
    {A,  5}, {A, 5}, {A,  5}, {A,  5}, REST,    REST,
    {E,  6}, {D, 6}, {FS, 5}, {FS, 5}, {GS, 5}, {GS, 5},
    {CS, 6}, {B, 5}, {D,  5}, {D,  5}, {E,  5}, {E,  5},
    {B,  5}, {A, 5}, {CS, 5}, {CS, 5}, {E,  5}, {E,  5},
    {A,  5}, {A, 5}, {A,  5}, {A,  5}, REST,
};

// Begin search tone.

static constexpr uint16_t SONG_BEGIN_SEARCH_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_BEGIN_SEARCH_NOTES[] = {
    {A, 4}, {B, 4}, {C, 5}, {C, 5}, {D, 5}, {E, 5}, {D, 6},
};

// Begin fast solve tone.

static constexpr uint16_t SONG_BEGIN_FAST_SOLVE_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_BEGIN_FAST_SOLVE_NOTES[] = {
    {C, 5},
    {C, 5},
    {C, 5},
    {D, 6},
};

// Begin slow solve tone.

static constexpr uint16_t SONG_BEGIN_SLOW_SOLVE_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_BEGIN_SLOW_SOLVE_NOTES[] = {
    {C, 5}, {C, 5}, {C, 5}, {D, 6}, {C, 5}, {D, 6},
};

// Begin other tone.

static constexpr uint16_t SONG_BEGIN_OTHER_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_BEGIN_OTHER_NOTES[] = {
    {C, 5},
    {C, 5},
    {C, 5},
    {C, 5},
};

// Armed tone.

static constexpr uint16_t SONG_ARMED_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_ARMED_NOTES[] = {
    {G, 4},
    REST,
    REST,
};

// Armed and triggering tone.

static constexpr uint16_t SONG_ARMED_AND_TRIGGERING_NOTE_LENGTH_MS = 250;

static constexpr Note SONG_ARMED_AND_TRIGGERING_NOTES[] = {
    {C, 5},
    REST,
};

static constexpr uint16_t SONG_WINDOWS_XP_SHUTDOWN_NOTE_LENGTH_MS = 350;

static constexpr Note SONG_WINDOWS_XP_SHUTDOWN_NOTES[] = {
    {G_SHARP, 5},
    {E_FLAT, 5},
    {B_FLAT, 4},
    {G_SHARP, 4},
    REST,
};

const AudioPlayer::SongHandle& AudioPlayer::get_song(Song song) const {
  switch (song) {
    using enum Song;
    default:
    case QUIET:
        static constexpr SongHandle QUIET_SONG_HANDLE = {};
      return QUIET_SONG_HANDLE;
    case HOME_DEPOT:
      static constexpr SongHandle HOME_DEPOT_SONG_HANDLE = {SONG_HOME_DEPOT_NOTES, SONG_HOME_DEPOT_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return HOME_DEPOT_SONG_HANDLE;
    case NOKIA:
      static constexpr SongHandle NOKIA_SONG_HANDLE = {SONG_NOKIA_NOTES, SONG_NOKIA_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS, false};
      return NOKIA_SONG_HANDLE;
    case WINDOWS_XP_SHUTDOWN:
      static constexpr SongHandle WINDOWS_XP_SHUTDOW_SONG_HANDLE = {SONG_WINDOWS_XP_SHUTDOWN_NOTES, SONG_WINDOWS_XP_SHUTDOWN_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS, false};
      return WINDOWS_XP_SHUTDOW_SONG_HANDLE;
    case STARTUP:
      static constexpr SongHandle STARTUP_SONG_HANDLE = {SONG_STARTUP_NOTES, SONG_STARTUP_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return STARTUP_SONG_HANDLE;
    case BLE_CONNECT:
      static constexpr SongHandle BLE_CONNECT_SONG_HANDLE = {SONG_BLE_CONNECT_NOTES, SONG_BLE_CONNECT_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return BLE_CONNECT_SONG_HANDLE;
    case BLE_DISCONNECT:
      static constexpr SongHandle BLE_DISCONNECT_SONG_HANDLE = {SONG_BLE_DISCONNECT_NOTES, SONG_BLE_DISCONNECT_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return BLE_DISCONNECT_SONG_HANDLE;
    case BEGIN_SEARCH:
      static constexpr SongHandle BEGIN_SEARCH_SONG_HANDLE = {SONG_BEGIN_SEARCH_NOTES, SONG_BEGIN_SEARCH_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return BEGIN_SEARCH_SONG_HANDLE;
    case BEGIN_FAST_SOLVE:
      static constexpr SongHandle BEGIN_FAST_SOLVE_SONG_HANDLE = {SONG_BEGIN_FAST_SOLVE_NOTES, SONG_BEGIN_FAST_SOLVE_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return BEGIN_FAST_SOLVE_SONG_HANDLE;
    case BEGIN_SLOW_SOLVE:
      static constexpr SongHandle BEGIN_SLOW_SOLVE_SONG_HANDLE = {SONG_BEGIN_SLOW_SOLVE_NOTES, SONG_BEGIN_SLOW_SOLVE_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return BEGIN_SLOW_SOLVE_SONG_HANDLE;
    case BEGIN_OTHER:
      static constexpr SongHandle BEGIN_OTHER_SONG_HANDLE = {SONG_BEGIN_OTHER_NOTES, SONG_BEGIN_OTHER_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return BEGIN_OTHER_SONG_HANDLE;
    case ARMED:
      static constexpr SongHandle ARMED_SONG_HANDLE = {SONG_ARMED_NOTES, SONG_ARMED_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return ARMED_SONG_HANDLE;
    case ARMED_TRIGGERING:
      static constexpr SongHandle ARMED_TRIGGERING_SONG_HANDLE = {SONG_ARMED_AND_TRIGGERING_NOTES, SONG_ARMED_AND_TRIGGERING_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS};
      return ARMED_TRIGGERING_SONG_HANDLE;
  }
}

// clang-format on

}  // namespace audio
