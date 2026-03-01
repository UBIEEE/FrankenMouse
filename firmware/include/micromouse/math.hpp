#pragma once

#include <cmath>
#include <limits>

template <typename T>
  requires std::is_floating_point_v<T>
constexpr bool float_equals(T x,
                            T y,
                            T epsilon = std::numeric_limits<T>::epsilon()) {
  return std::abs(x - y) < epsilon;
}
