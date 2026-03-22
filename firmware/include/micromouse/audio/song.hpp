#pragma once

#include <cstdint>

namespace audio {

enum class Song : uint8_t {
  QUIET = 0,

  // 1-127 Songs

  HOME_DEPOT = 1,
  NOKIA,
  WINDOWS_XP_SHUTDOWN,

  // 128-255 Other noises

  STARTUP = 128,
  BLE_CONNECT,
  BLE_DISCONNECT,

  BEGIN_SEARCH,
  BEGIN_FAST_SOLVE,
  BEGIN_SLOW_SOLVE,
  BEGIN_OTHER,

  ARMED,
  ARMED_TRIGGERING,
};

const char* song_to_string(Song song);

}  // namespace audio
