#pragma once

#include <cstdint>

enum class RobotSong : uint8_t {
  NONE = 0,

  // 1-127 Songs

  HOME_DEPOT = 1,
  NOKIA,

  _SONG_COUNT,

  // 128-255 Other noises

  STARTUP = 128,
  BLE_CONECT,
  BLE_DISCONECT,

  BEGIN_SEARCH,
  BEGIN_FAST_SOLVE,
  BEGIN_SLOW_SOLVE,
  BEGIN_OTHER,

  ARMED,
  ARMED_TRIGGERING,

  _COUNT,
};