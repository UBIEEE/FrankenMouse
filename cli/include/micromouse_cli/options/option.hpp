#pragma once

#include <span>
#include <string_view>

class OptionName {
  const char* m_names[3] = {};

 public:
  OptionName(const char* primary) : OptionName(primary, nullptr, nullptr) {}

  OptionName(const char* primary, const char* secondary)
      : OptionName(primary, secondary, nullptr) {}

  OptionName(const char* primary, const char* secondary, const char* tertiary) {
    m_names[0] = primary;
    m_names[1] = secondary;
    m_names[2] = tertiary;
  }

  std::span<const char* const> names() const { return std::span(m_names, 3); }

  constexpr const char* primary() const { return m_names[0]; }
  constexpr const char* secondary() const { return m_names[1]; }
  constexpr const char* tertiary() const { return m_names[2]; }
};

struct Option {
  int id;
  OptionName name;
  bool requires_value;
  const char* value_text;
  const char* help;
};
