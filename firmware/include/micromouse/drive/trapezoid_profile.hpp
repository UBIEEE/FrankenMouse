// Adapted from
// https://github.com/wpilibsuite/allwpilib/blob/77dfad97c66fd4922fb19fd7f805c96357ea13cb/wpimath/src/main/native/include/frc/trajectory/TrapezoidProfile.h
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <type_traits>

#include <units/math.h>
#include <units/time.h>

namespace drive {

/**
 * A trapezoid-shaped velocity profile.
 */
template <class Distance>
class TrapezoidProfile {
 public:
  using Distance_t = units::unit_t<Distance>;
  using Velocity = units::compound_unit<Distance, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<Velocity>;
  using Acceleration = units::compound_unit<Velocity, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<Acceleration>;

  class Constraints {
   public:
    Velocity_t max_velocity{0};
    Acceleration_t max_acceleration{0};

    constexpr Constraints() = default;

    constexpr Constraints(Velocity_t max_velocity, Acceleration_t max_acceleration)
        : max_velocity(max_velocity), max_acceleration(max_acceleration) {}

    constexpr bool is_valid() const {
      return max_velocity > Velocity_t{0} && max_acceleration > Acceleration_t{0};
    }
    constexpr explicit operator bool() const { return is_valid(); }
  };

  class State {
   public:
    Distance_t position{0};
    Velocity_t velocity{0};

    constexpr bool operator==(const State&) const = default;
  };

  class SampledState {
   public:
    Distance_t position{0};
    Velocity_t velocity{0};
    Distance_t distance{0};

    constexpr SampledState() = default;
    constexpr SampledState(const State& state, Distance_t distance)
        : position(state.position), velocity(state.velocity), distance(distance) {}
  };

  constexpr TrapezoidProfile() = default;

  constexpr TrapezoidProfile(const TrapezoidProfile&) = default;
  constexpr TrapezoidProfile& operator=(const TrapezoidProfile&) = default;
  constexpr TrapezoidProfile(TrapezoidProfile&&) = default;
  constexpr TrapezoidProfile& operator=(TrapezoidProfile&&) = default;

  /**
   * Configure the profile to travel between two states, subject to given constraints.
   *
   * @param initial The initial state of the profile.
   * @param final The final state of the profile.
   * @param constraints The constraints of the profile.
   */
  constexpr void configure(const State& initial, const State& final, const Constraints& constraints) {
    if (!constraints) {
      reset();
      return;
    }

    m_direction = should_flip_acceleration(initial, final) ? -1 : 1;
    m_initial = direct(initial);
    m_final = direct(final);
    m_final_undirected = final;
    m_constraints = constraints;

    if (units::math::abs(m_initial.velocity) > m_constraints.max_velocity) {
      m_initial.velocity = units::math::copysign(m_constraints.max_velocity, m_initial.velocity);
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    units::second_t cutoff_begin = m_initial.velocity / m_constraints.max_acceleration;
    Distance_t cutoff_dist_begin = cutoff_begin * cutoff_begin * m_constraints.max_acceleration / 2.0;

    units::second_t cutoff_end = m_final.velocity / m_constraints.max_acceleration;
    Distance_t cutoff_dist_end = cutoff_end * cutoff_end * m_constraints.max_acceleration / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    Distance_t full_trapezoid_dist =
        cutoff_dist_begin + (m_final.position - m_initial.position) + cutoff_dist_end;
    units::second_t acceleration_time = m_constraints.max_velocity / m_constraints.max_acceleration;

    Distance_t full_speed_dist =
        full_trapezoid_dist - acceleration_time * acceleration_time * m_constraints.max_acceleration;

    // Handle the case where the profile never reaches full speed
    if (full_speed_dist < Distance_t{0}) {
      acceleration_time = units::math::sqrt(full_trapezoid_dist / m_constraints.max_acceleration);
      full_speed_dist = Distance_t{0};
    }

    m_end_accel = acceleration_time - cutoff_begin;
    m_end_full_speed = m_end_accel + full_speed_dist / m_constraints.max_velocity;
    m_end_decel = m_end_full_speed + acceleration_time - cutoff_end;
  }

  /**
   * Configure the profile to travel a given distance, starting at a given initial velocity and ending at a
   * given final velocity, subject to given constraints.
   *
   * @param distance The distance to travel.
   * @param initial_velocity The initial velocity of the profile.
   * @param final_velocity The final velocity of the profile.
   * @param constraints The constraints of the profile.
   */
  constexpr void configure(Distance_t distance,
                           Velocity_t initial_velocity,
                           Velocity_t final_velocity,
                           const Constraints& constraints) {
    State initial_state = {.position = Distance_t{0}, .velocity = initial_velocity};
    State final_state = {.position = distance, .velocity = final_velocity};
    configure(initial_state, final_state, constraints);
  }

  /**
   * Configure the profile to travel between the final state of the last profile and a given final state,
   * subject to given constraints.
   *
   * @param final_state The final state of the profile.
   * @param constraints The constraints of the profile.
   */
  constexpr void configure(const State& final_state, const Constraints& constraints) {
    configure(m_final_undirected, final_state, constraints);
  }

  /**
   * Configure the profile to travel a given distance, starting at the final state of the last profile and
   * ending at a given final velocity, subject to given constraints.
   *
   * @param distance The distance to travel.
   * @param final_velocity The final velocity of the profile.
   * @param constraints The constraints of the profile.
   */
  constexpr void configure(Distance_t distance, Velocity_t final_velocity, const Constraints& constraints) {
    State initial_state = m_final_undirected;
    State final_state = {.position = m_final_undirected.position + distance, .velocity = final_velocity};
    configure(initial_state, final_state, constraints);
  }

  /**
   * Reset the profile.
   */
  constexpr void reset() {
    m_initial = m_final = m_final_undirected = {};
    m_constraints = {};

    m_end_accel = m_end_full_speed = m_end_decel = 0_s;
  }

  /**
   * Calculates the position and velocity for the profile at a time t.
   *
   * @param t How long to advance the initial state towards the final state.
   * @return The state of the profile at the given time.
   */
  constexpr SampledState sample(units::second_t t) const {
    State result = m_initial;

    if (t < m_end_accel) {
      result.velocity += t * m_constraints.max_acceleration;
      result.position += (m_initial.velocity + t * m_constraints.max_acceleration / 2.0) * t;
    } else if (t < m_end_full_speed) {
      result.velocity = m_constraints.max_velocity;
      result.position +=
          (m_initial.velocity + m_end_accel * m_constraints.max_acceleration / 2.0) * m_end_accel +
          m_constraints.max_velocity * (t - m_end_accel);
    } else if (t <= m_end_decel) {
      result.velocity = m_final.velocity + (m_end_decel - t) * m_constraints.max_acceleration;
      units::second_t time_left = m_end_decel - t;
      result.position = m_final.position -
                        (m_final.velocity + time_left * m_constraints.max_acceleration / 2.0) * time_left;
    } else {
      result = m_final;
    }

    Distance_t distance = m_final.position - m_initial.position;
    return SampledState(direct(result), distance);
  }

  /**
   * Returns the initial state of the profile.
   *
   * @return The initial state of the profile.
   */
  const State& initial_state() const { return m_initial; }

  /**
   * Returns the final state of the profile.
   *
   * @return The final state of the profile.
   */
  const State& final_state() const { return m_final_undirected; }

  /**
   * Returns the constraints of the profile.
   *
   * @return The constraints of the profile.
   */
  const Constraints& constraints() const { return m_constraints; }

  /**
   * Returns the distance of the profile.
   *
   * @return The distance of the profile.
   */
  const Distance_t distance() const { return m_final_undirected.position - m_initial.position; }

  /**
   * Returns the time left until a target distance in the profile is reached.
   *
   * @param target The target distance.
   * @return The time left until a target distance in the profile is reached, or zero if no goal was set.
   */
  constexpr units::second_t time_left_until(Distance_t target) const {
    if (!m_constraints) {
      return 0_s;
    }

    Distance_t position = m_initial.position * m_direction;
    Velocity_t velocity = m_initial.velocity * m_direction;

    units::second_t end_accel = m_end_accel * m_direction;
    units::second_t end_full_speed = m_end_full_speed * m_direction - end_accel;

    if (target < position) {
      end_accel *= -1.0;
      end_full_speed *= -1.0;
      velocity *= -1.0;
    }

    end_accel = units::math::max(end_accel, 0_s);
    end_full_speed = units::math::max(end_full_speed, 0_s);

    const Acceleration_t acceleration = m_constraints.max_acceleration;
    const Acceleration_t deceleration = -m_constraints.max_acceleration;

    Distance_t dist_to_target = units::math::abs(target - position);

    if (dist_to_target < Distance_t{1e-6}) {
      return 0_s;
    }

    Distance_t accel_dist = velocity * end_accel + 0.5 * acceleration * end_accel * end_accel;

    Velocity_t decel_velocity;
    if (end_accel > 0_s) {
      decel_velocity =
          units::math::sqrt(units::math::abs(velocity * velocity + 2 * acceleration * accel_dist));
    } else {
      decel_velocity = velocity;
    }

    Distance_t full_speed_dist = m_constraints.max_velocity * end_full_speed;
    Distance_t decel_dist;

    if (accel_dist > dist_to_target) {
      accel_dist = dist_to_target;
      full_speed_dist = Distance_t{0};
      decel_dist = Distance_t{0};
    } else if (accel_dist + full_speed_dist > dist_to_target) {
      full_speed_dist = dist_to_target - accel_dist;
      decel_dist = Distance_t{0};
    } else {
      decel_dist = dist_to_target - full_speed_dist - accel_dist;
    }

    units::second_t accel_time =
        (-velocity +
         units::math::sqrt(units::math::abs(velocity * velocity + 2 * acceleration * accel_dist))) /
        acceleration;

    units::second_t decel_time =
        (-decel_velocity + units::math::sqrt(units::math::abs(decel_velocity * decel_velocity +
                                                              2 * deceleration * decel_dist))) /
        deceleration;

    units::second_t full_speed_time = full_speed_dist / m_constraints.max_velocity;

    return accel_time + full_speed_time + decel_time;
  }

  /**
   * Returns the total time the profile takes to reach the goal.
   *
   * @return The total time the profile takes to reach the goal, or zero if no goal was set.
   */
  constexpr units::second_t total_time() const { return m_end_decel; }

  /**
   * Returns true if the profile has reached the goal.
   *
   * The profile has reached the goal if the time since the profile started has exceeded the profile's total
   * time.
   *
   * @param t The time since the beginning of the profile.
   * @return True if the profile has reached the goal.
   */
  constexpr bool is_finished(units::second_t t) const { return t >= total_time(); }

 private:
  /**
   * Returns true if the profile inverted.
   *
   * The profile is inverted if goal position is less than the initial position.
   *
   * @param initial The initial state (usually the initial state).
   * @param goal The desired state when the profile is complete.
   */
  static constexpr bool should_flip_acceleration(const State& initial, const State& final) {
    return initial.position > final.position;
  }

  // Flip the sign of the velocity and position if the profile is inverted
  constexpr State direct(const State& in) const {
    State result = in;
    result.position *= m_direction;
    result.velocity *= m_direction;
    return result;
  }

  // The direction of the profile, either 1 for forwards or -1 for inverted
  int m_direction = 1;

  Constraints m_constraints;
  State m_initial;
  State m_final;
  State m_final_undirected;

  units::second_t m_end_accel = 0_s;
  units::second_t m_end_full_speed = 0_s;
  units::second_t m_end_decel = 0_s;
};

}  // namespace drive
