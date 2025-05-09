#pragma once

#include <concepts>
#include <span>
#include <string>
#include <string_view>
#include <vector>

using CommandArguments = std::vector<std::string>;

enum class CommandProcessResult {
  DONE,
  CONTINUE,
  EXIT_ALL,
};

class Command {
 protected:
  const CommandArguments m_args;

  Command(CommandArguments args) : m_args(args) {}

 public:
  virtual ~Command() = default;

  /**
   * @brief Process the command. Return true if done.
   *
   * @return True if done, false if it should keep running.
   */
  virtual CommandProcessResult process() { return CommandProcessResult::DONE; }

  virtual bool is_done() const { return false; }

  explicit operator bool() const { return is_done(); }
};

template <typename T>
concept CommandType_ConstructibleFromArgs =
    std::constructible_from<T, CommandArguments>;

template <typename T>
concept CommandType_ConstructibleFromArgsAndBLEManager =
    std::constructible_from<T, CommandArguments, class BLEManager&>;

/**
 * @brief Base concept that all command types must satisfy.
 *        1. Must be derived from Command.
 *        2. Must be constructible from CommandArguments and optionally
 *           BLEManager&.
 *        3. Must have a static method `name()` that returns the command's name
 *           (const char*).
 */
template <typename T>
concept CommandType = std::derived_from<T, Command> &&
                      (CommandType_ConstructibleFromArgs<T> ||
                       CommandType_ConstructibleFromArgsAndBLEManager<T>) &&
                      requires(T t) {
                        { T::name() } -> std::same_as<const char*>;
                      };

/**
 * @brief Commands may implement a static method `options()` that returns a span
 *        of strings. This is used by the prompt for hints and tab completion.
 */
template <typename T>
concept CommandType_WithOptions = CommandType<T> && requires(T t) {
  { T::options() } -> std::same_as<std::span<const char* const>>;
};

/**
 * @brief Commands may implement a static method `can_accept_file_paths()` that
 *        returns a bool indicating whether the command can accept file paths as
 *        arguments. This is used by the prompt for hints, tab completion, and
 *        syntax highlighting.
 */
template <typename T>
concept CommandType_WithCanAcceptFilePaths = CommandType<T> && requires(T t) {
  { T::can_accept_file_paths() } -> std::same_as<bool>;
};
