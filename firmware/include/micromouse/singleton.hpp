#pragma once

// Singleton base class.
template <typename T>
class Singleton {
 public:
  static constexpr T& get() {
    static T instance;
    return instance;
  }

  Singleton(const Singleton&) = delete;
  Singleton(Singleton&&) = delete;
  Singleton& operator=(const Singleton&) = delete;
  Singleton& operator=(Singleton&&) = delete;

 protected:
  Singleton() = default;
};
