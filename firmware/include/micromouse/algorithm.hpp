#pragma once

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

// Binary insertion into vector; std::log(n) efficiency.
template <typename T>
typename std::vector<T>::iterator insert_sorted(std::vector<T>& vec, T const& item) {
  return vec.insert(std::upper_bound(vec.begin(), vec.end(), item), item);
}
