// Adapted from
// https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpimath/src/main/native/include/frc/filter/MedianFilter.h
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <vector>

#include <micromouse/algorithm.hpp>
#include <micromouse/circular_buffer.hpp>

/**
 * A class that implements a moving-window median filter. Useful for reducing measurement noise, especially
 * with processes that generate occasional, extreme outliers (such as values from vision processing, LIDAR, or
 * ultrasonic sensors).
 *
 * @tparam T Buffer element type.
 * @tparam N Maximum number of buffer elements.
 */
template <class T, size_t N>
class MedianFilter {
 public:
  /**
   * Creates a new MedianFilter.
   */
  constexpr MedianFilter() = default;

  /**
   * Calculates the moving-window median for the next value of the input stream.
   *
   * @param next The next input value.
   * @return The median of the moving window, updated to include the next value.
   */
  constexpr T calculate(T next) {
    // Insert next value at proper point in sorted array
    insert_sorted(m_ordered_values, next);

    size_t cur_size = m_ordered_values.size();

    // If buffer is at max size, pop element off of end of circular buffer and remove from ordered list
    if (cur_size > N) {
      m_ordered_values.erase(
          std::find(m_ordered_values.begin(), m_ordered_values.end(), m_value_buffer.pop_back()));
      --cur_size;
    }

    // Add next value to circular buffer
    m_value_buffer.push_front(next);

    if (cur_size % 2 != 0) {
      // If size is odd, return middle element of sorted list
      return m_ordered_values[cur_size / 2];
    } else {
      // If size is even, return average of middle elements
      return (m_ordered_values[cur_size / 2 - 1] + m_ordered_values[cur_size / 2]) / 2.0;
    }
  }

  /**
   * Returns the last value calculated by the MedianFilter.
   *
   * @return The last value.
   */
  constexpr T last_value() const { return m_value_buffer.front(); }

  /**
   * Resets the filter, clearing the window of all elements.
   */
  constexpr void reset() {
    m_ordered_values.clear();
    m_value_buffer.reset();
  }

 private:
  CircularBuffer<T, N> m_value_buffer;
  std::vector<T> m_ordered_values;
};
