#pragma once

#include <micromouse_cli/commands/command.hpp>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>

using MakeCommandFunc = Command* (*)(CommandArguments);

class Prompt final {
  std::string m_prompt_text = "micromouse";

  struct CommandInfo {
    std::span<const char*> options;
    bool can_accept_file_paths;
    MakeCommandFunc make_command_func;
  };

  std::unordered_map<std::string, CommandInfo> m_commands;
  using CommandIterator = decltype(m_commands)::const_iterator;

 public:
  Prompt(std::string_view text = "") {
    configure();
    if (!text.empty())
      set_text(text);
  }
  ~Prompt() = default;

  void set_text(std::string_view prompt_text) { m_prompt_text = prompt_text; }
  void set_text(const std::string& prompt_text) { m_prompt_text = prompt_text; }
  std::string_view get_prompt_text() const { return m_prompt_text; }

  /**
   * @brief Adds a command to the list of available input commands for this
   *        prompt.
   *
   * @tparam T Type of the command to register. Must conform to the CommandType
   *           concept.
   */
  template <CommandType T>
  void register_command() {
    const char* name = T::name();

    std::span<const char*> options;
    if constexpr (CommandType_WithOptions<T>) {
      options = T::options();
    }

    bool can_accept_file_paths = false;
    if constexpr (CommandType_WithCanAcceptFilePaths<T>) {
      can_accept_file_paths = T::can_accept_file_paths();
    }

    register_command(
        name, options, can_accept_file_paths,
        [](CommandArguments args) -> Command* { return new T(args); });
  }

  void register_command(const char* name,
                        std::span<const char*> options,
                        bool can_accept_file_paths,
                        MakeCommandFunc make_command);

  /**
   * @brief Prompts the user for input and returns an instance of the
   *        invoked command.
   *
   * @return A heap-allocated instance of the invoked command, or nullptr if an
   *         exit signal was caught or an error occurred. User must free the
   *         returned object.
   */
  Command* readline();

 private:
  static std::optional<std::string> get_history_filename();

  void configure();

  void enable_history();
  void configure_colors();
  void configure_completer();
  void configure_highlighter();
  void enable_auto_completion();
  void enable_inline_hints();

  void completer(struct ic_completion_env_s* cenv, const char* input);
  void word_completer(struct ic_completion_env_s* cenv, const char* word);
  void complete_command_name(struct ic_completion_env_s* cenv,
                             const char* word);
  void complete_command_options(struct ic_completion_env_s* cenv,
                                CommandIterator command_it,
                                const char* word);

  void highlighter(struct ic_highlight_env_s* henv, const char* input);

  struct CommandInvocation {
    CommandIterator command;

    // index 0 is the command name, 1+ are arguments
    CommandArguments args;
  };

  CommandInvocation parse_command_invocation(const char* input);

  /**
   * @brief Parses the first word of the input string, matches it against the
   *        registered commands, and returns the corresponding command iterator
   *        if found.
   *
   * @param input Pointer to the input string (cannot be nullptr).
   *
   * @return Iterator to the command if found, or end() if not found.
   */
  CommandIterator match_current_command(const char* input);

 private:
  /**
   * @brief Return a pointer to the beginning of the next word in the input
   *        string. The length of the word is stored in word_len.
   *
   * @param input     Pointer to the input string (cannot be nullptr). The
   *                  pointer is updated to point to the character following the
   *                  word.
   * @param word_len  Length of the word found (cannot be nullptr).
   * @param is_quoted (Optional output) Set to true if the word was surrounded
   *                  by quotes (single or double)
   *
   * @return A pointer to the beginning of the next word in the input string, or
   *         nullptr if no word is found.
   */
  static const char* get_next_word(const char** input,
                                   size_t* word_len,
                                   bool* is_quoted = nullptr);

  static const char* get_first_word(const char* input, size_t* word_len) {
    return get_next_word(&input, word_len);
  }

  /**
   * @brief Returns true if the cursor is positioned in the first word of the
   *        input string.
   *
   * @param input  Null-terminated input string.
   * @param cursor Element index of the cursor in the input string.
   */
  static bool is_cursor_in_first_word(const char* input, long cursor);
};
