#pragma once

#include <micromouse_cli/options/option.hpp>

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

  /**
   * @brief Information about the command, used by the prompt for hints and tab
   *        completion, and help descriptions.
   */
  struct PromptInfo {
    std::span<const Option> options = {};
    const std::vector<std::string>* non_options = nullptr;  // Janky, ok for now

    bool can_accept_file_paths = false;

    const char* usage_text = nullptr;
    const char* description_text = nullptr;
  };

  /**
   * @brief Emit usage information for the command.
   *
   * @param command_name     The name of the command (how it is invoked).
   * @param usage_text       How the command should be invoked, ex.
   *                         "command [options] [files...]"
   * @param description_text A short blurb about the command.
   * @param options          A span of options that the command accepts.
   */
  static void help(const char* command_name,
                   const char* usage_text,
                   const char* description_text,
                   std::span<const Option> options);

  static void help(const char* command_name, const PromptInfo& prompt_info) {
    help(command_name, prompt_info.usage_text, prompt_info.description_text,
         prompt_info.options);
  }
};

/**
 * @brief Overrides the is_done() method to always return true.
 *        This is useful for commands that do not need to be processed.
 */
#define COMMAND_INSTANT()         \
 public:                          \
  bool is_done() const override { \
    return true;                  \
  }

/**
 * @brief Implement the required static name() method for a command.
 */
#define COMMAND_NAME(name_str) \
 public:                       \
  static const char* name() {  \
    return name_str;           \
  }

/**
 * @brief Implements the static prompt_info() method for a command.
 */
#define COMMAND_PROMPT_INFO(usage_text_str, description_text_str, \
                            options_span)                         \
 public:                                                          \
  static PromptInfo prompt_info() {                               \
    return PromptInfo{                                            \
        .options = options_span,                                  \
        .usage_text = usage_text_str,                             \
        .description_text = description_text_str,                 \
    };                                                            \
  }

/**
 * @brief Implements the static name() and prompt_info() methods for a command.
 *
 */
#define COMMAND_NAME_AND_PROMPT_INFO(name_str, usage_text_str,           \
                                     description_text_str, options_span) \
  COMMAND_NAME(name_str)                                                 \
  COMMAND_PROMPT_INFO(usage_text_str, description_text_str, options_span)

template <typename T>
concept CommandType_ConstructibleFromArguments =
    std::constructible_from<T, CommandArguments>;

template <typename T>
concept CommandType_ConstructibleFromArgumentsAndBLEManager =
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
concept CommandType =
    std::derived_from<T, Command> &&
    (CommandType_ConstructibleFromArguments<T> ||
     CommandType_ConstructibleFromArgumentsAndBLEManager<T>) &&
    requires(T t) {
      { T::name() } -> std::same_as<const char*>;
    };

template <typename T>
concept CommandType_WithPromptInfo = CommandType<T> && requires(T t) {
  { T::prompt_info() } -> std::same_as<Command::PromptInfo>;
};
