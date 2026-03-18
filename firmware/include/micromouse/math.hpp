#pragma once

#include <gcem.hpp>
#include <cmath>
#include <limits>

template <typename T>
  requires std::is_floating_point_v<T>
constexpr bool float_equals(T x, T y, T epsilon = std::numeric_limits<T>::epsilon()) {
  return gcem::abs(x - y) <= epsilon;
}

/**
 * Approximate the integral of f from a to b using the trapezoidal rule with n subdivisions.
 *
 * @param f The function to integrate.
 * @param a The lower limit of integration.
 * @param b The upper limit of integration.
 * @param n The number of subdivisions to use (default is 100).
 *
 * @return The approximate integral of f from a to b.
 */
constexpr float approximate_integral(float (*f)(float), float a, float b, size_t n = 100) {
  float h = (b - a) / n;
  float sum = 0.5f * (f(a) + f(b));

  for (size_t i = 1; i < n; ++i) {
    sum += f(a + i * h);
  }
  return sum * h;
}
