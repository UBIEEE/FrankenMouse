#include <micromouse/audio/audio_player.hpp>

#include <micromouse/robot.h>

using namespace audio;
using enum NoteInOctave;

//clang-format off

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

const AudioPlayer::SongHandle AudioPlayer::m_songs[] = {
    // NONE
    {},

    // STARTUP
    {SONG_STARTUP_NOTES, SONG_STARTUP_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // BLE_CONNECT
    {SONG_BLE_CONNECT_NOTES,
     SONG_BLE_CONNECT_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // BLE_DISCONNECT
    {SONG_BLE_DISCONNECT_NOTES,
     SONG_BLE_DISCONNECT_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // HOME_DEPOT
    {SONG_HOME_DEPOT_NOTES,
     SONG_HOME_DEPOT_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // NOKIA
    {SONG_NOKIA_NOTES, SONG_NOKIA_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS,
     false},

    // BEGIN_SEARCH
    {SONG_BEGIN_SEARCH_NOTES,
     SONG_BEGIN_SEARCH_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // BEGIN_FAST_SOLVE
    {SONG_BEGIN_FAST_SOLVE_NOTES,
     SONG_BEGIN_FAST_SOLVE_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // BEGIN_SLOW_SOLVE
    {SONG_BEGIN_SLOW_SOLVE_NOTES,
     SONG_BEGIN_SLOW_SOLVE_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // BEGIN_OTHER
    {SONG_BEGIN_OTHER_NOTES,
     SONG_BEGIN_OTHER_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // ARMED
    {SONG_ARMED_NOTES, SONG_ARMED_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},

    // ARMED_AND_TRIGGERING
    {SONG_ARMED_AND_TRIGGERING_NOTES,
     SONG_ARMED_AND_TRIGGERING_NOTE_LENGTH_MS / ROBOT_UPDATE_PERIOD_MS},
};

// clang-format on
