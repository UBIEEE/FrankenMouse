#pragma once

#include <micromouse_cli/options/option.hpp>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class ArgumentParser final {
  std::span<const std::string> m_args;
  std::span<const Option> m_options;

  std::vector<std::string_view> m_non_option_args;

  std::unordered_set<int> m_parsed_options;
  std::unordered_map<int, std::string_view> m_parsed_option_values;

  bool m_has_error = false;

 public:
  ArgumentParser(std::span<const std::string> args,
                 std::span<const Option> options)
      : m_args(args), m_options(options) {
    parse();
  }

  std::span<const std::string> args() const { return m_args; }
  std::span<const Option> options() const { return m_options; }

  const std::unordered_set<int>& parsed_options() const {
    return m_parsed_options;
  }
  const std::unordered_map<int, std::string_view>& parsed_option_values()
      const {
    return m_parsed_option_values;
  }

  std::span<const std::string_view> non_option_args() const {
    return m_non_option_args;
  }

  bool has_error() const { return m_has_error; }

 private:
  void parse();

  enum Error {
    NONE = 0,
    INVALID_OPTION,
    MISSING_VALUE,
    DUPLICATE_OPTION,
  };

  Error parse_option(size_t* index);

  size_t match_option(const Option& option, std::string_view arg) const;
};
