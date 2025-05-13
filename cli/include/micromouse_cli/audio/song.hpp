#pragma once

#include <cstdint>

enum class Song : uint8_t {
  NONE = 0,
  STARTUP,
  BLE_CONECT,
  BLE_DISCONECT,

  HOME_DEPOT = 4,
  NOKIA = 5,

  BEGIN_SEARCH,
  BEGIN_FAST_SOLVE,
  BEGIN_SLOW_SOLVE,
  BEGIN_OTHER,

  ARMED,
  ARMED_TRIGGERING,

  _COUNT,
};
