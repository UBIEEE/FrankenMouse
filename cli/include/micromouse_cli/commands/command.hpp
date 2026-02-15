#pragma once

#include <micromouse_cli/options/option.hpp>

#include <concepts>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include <cstdio>
#include <functional>

using CommandArguments = std::vector<std::string>;

enum class CommandStatus {
  DONE,
  CONTINUING,
  EXIT_ALL,
};

class Command {
 protected:
  const CommandArguments m_args;

  explicit Command(CommandArguments args) : m_args(args) {}

 public:
  virtual ~Command() = default;

  /**
   * Initialize the command. Called once before processing begins. Perform any feedback operations here
   * instead of the constructor.
   */
  virtual void init() {}

  /**
   * Process the command, called continuously as long as status() returns CONTINUING.
   */
  virtual void process() {}

  /**
   * End the command, called once after processing ends.
   *
   * @param interrupted Whether the command was interrupted (e.g. by a signal or disconnect) or ended normally
   *                    (status() returned something other than CONTINUING).
   */
  virtual void end(bool interrupted) { (void)interrupted; }

  /**
   * Get the status of the command, called continuously to determine whether to continue processing.
   *
   * @return The status
   */
  virtual CommandStatus status() const = 0;

  /**
   * Information about the command, used by the prompt for hints and tab completion, and help descriptions.
   */
  struct PromptInfo {
    const char* usage_text = nullptr;
    const char* short_description_text = nullptr;
    const char* long_description_text = nullptr;

    std::span<const Option> options = {};

    const char* non_options_title = nullptr;
    std::span<const std::string> non_options = {};

    bool can_accept_file_paths = false;

    // Instead of calling the default help function, use this function instead to emit usage information.
    std::function<void(FILE*)> custom_help_func = nullptr;

    // When using the default help function, this function will be called after to display any additional
    // information.
    std::function<void(FILE*)> supplemental_help_func = nullptr;
  };

  /**
   * Emit usage information for the command. Calls the custom_help_func if it is set, otherwise it uses the
   * information from prompt_info to construct the help message.
   *
   * @param command_name The name of the command.
   * @param prompt_info  The prompt information for the command.
   */
  static void help(const char* command_name, const PromptInfo& prompt_info, FILE* stream);

  static void usage(const char* command_name, const PromptInfo& prompt_info, FILE* stream);
};

template <typename T>
concept CommandType_ConstructibleFromArguments = std::constructible_from<T, CommandArguments>;

template <typename T>
concept CommandType_ConstructibleFromArgumentsAndCommunicationManager =
    std::constructible_from<T, CommandArguments, class CommunicationManager&>;

/**
 * Base concept that all command types must satisfy.
 * 1. Must be derived from Command.
 * 2. Must be constructible from CommandArguments and optionally CommunicationManager&.
 * 3. Must have a static method `name()` that returns the command's name (const char*).
 */
template <typename T>
concept CommandType = std::derived_from<T, Command> &&
                      (CommandType_ConstructibleFromArguments<T> ||
                       CommandType_ConstructibleFromArgumentsAndCommunicationManager<T>) &&
                      requires(T t) {
                        { T::name() } -> std::same_as<const char*>;
                      };

template <typename T>
concept CommandType_WithPromptInfo = CommandType<T> && requires(T t) {
  { T::prompt_info() } -> std::same_as<Command::PromptInfo>;
};
