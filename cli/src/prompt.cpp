#include <micromouse_cli/prompt.hpp>

#include <micromouse_cli/diagnostics.hpp>

#include <isocline.h>
#include <cassert>
#include <filesystem>
#include <thread>

using namespace std::chrono_literals;

#define HELP_COMMAND_INDENT 4
#define HELP_COMMAND_DESCRIPTION_COLUMN 20

/**
 * @brief The help command is special because it shows usage information for
 *        every command registered with the prompt.
 */
class HelpCommand final : public Command {
  COMMAND_NAME("help");

 public:
  HelpCommand(
      const CommandArguments args,
      const std::unordered_map<std::string, Prompt::CommandInfo>& commands)
      : Command(args) {
    if (args.size() > 1) {
      Prompt::CommandIterator it = commands.find(args[1]);
      if (it != commands.end()) {
        const auto& [name, info] = *it;
        const Command::PromptInfo& prompt_info = info.prompt_info;
        Command::help(name.c_str(), prompt_info);
        return;
      }
    }

    // clang-format off
    puts("Usage: help [command]");
    puts("");
    puts("What you see here is a shell for controlling the MicroMouse.");
    puts("");
    puts("When this shell is running, a BLE connection is established to the MicroMouse.");
    puts("");
    puts("There are a number of commands for controlling and reading data from the\n"
         "MicroMouse. You can display detailed usage information for a command by passing\n"
         "the command name as an argument to the help command.");
    puts("");
    // clang-format on

    puts("Commands:");
    for (const auto& [name, info] : commands) {
      printf("%*s%s", HELP_COMMAND_INDENT, "", name.c_str());
      const char* description = info.prompt_info.description_text;
      if (!description) {
        putchar('\n');
        continue;
      }

      int remaining_space = HELP_COMMAND_DESCRIPTION_COLUMN -
                            static_cast<int>(name.length()) -
                            HELP_COMMAND_INDENT;
      if (remaining_space > 0) {
        printf("%*s", remaining_space, "");
      } else {
        printf("\n%*s", HELP_COMMAND_DESCRIPTION_COLUMN, "");
      }
      printf("%s\n", description);
    }
  }

  bool is_done() const override { return true; }
};

void Prompt::register_command(const char* name,
                              Command::PromptInfo prompt_info,
                              MakeCommandFunc make_command) {
  m_command_names.emplace_back(name);
  m_commands.emplace(std::string(name), CommandInfo{prompt_info, make_command});
}

Prompt::Result Prompt::readline(Command** command) {
  assert(command != nullptr);
  *command = nullptr;

  if (!m_ble_manager.is_connected())
    return Result::BLE_NOT_CONNECTED;

  m_ble_disconnect_stop = false;

  CommandInvocation result;

  bool repeat;
  do {
    char* input = ic_readline("micromouse");
    if (!input) {
      // Determine if the prompt was stopped due to ble disconnection or signal
      // caught by isocline.
      return m_ble_disconnect_stop ? Result::BLE_NOT_CONNECTED
                                   : Result::SIGNAL_OR_ERROR;
    }

    result = parse_command_invocation(input);
    free(input);

    repeat = (result.command == m_commands.end());
    if (repeat && !result.args.empty()) {
      report_error(nullptr, "unknown command: '%s'", result.args[0].c_str());
    }
  } while (repeat);

  const auto& [name, info] = *result.command;
  *command = info.make_command_func(std::move(result.args));

  return Result::COMMAND;
}

std::optional<std::string> Prompt::get_history_filename() {
  const char* home_dir = getenv("HOME");
  if (!home_dir)
    return std::nullopt;

  return std::string(home_dir) + "/mm_history.txt";
}

void Prompt::configure() {
  add_help_command();
  configure_ble_disconnect_callback();
  enable_history();
  configure_colors();
  configure_completer();
  configure_highlighter();
  enable_auto_completion();
  enable_inline_hints();
}

void Prompt::add_help_command() {
  Command::PromptInfo prompt_info;
  prompt_info.usage_text = "help [command]";
  prompt_info.description_text = "Show usage information for a command.";
  prompt_info.non_options = &m_command_names;

  MakeCommandFunc make_command_func = [&](CommandArguments args) -> Command* {
    return new HelpCommand(args, m_commands);
  };

  register_command(HelpCommand::name(), prompt_info, make_command_func);
}

void Prompt::configure_ble_disconnect_callback() {
  // Stop the prompt when the BLE connection is dropped.
  m_ble_manager.set_on_disconnect_callback([&]() {
    ic_async_stop();
    m_ble_disconnect_stop = true;
    std::this_thread::sleep_for(1ms);  // Nothing to see here...
  });
}

void Prompt::enable_history() {
  auto filename = get_history_filename();
  const char* filename_cstr = filename ? filename->c_str() : nullptr;

  ic_set_history(filename_cstr, -1);
}

void Prompt::configure_colors() {
  ic_style_def("ic-prompt", "bold ansi-purple");

  // Custom styles
  ic_style_def("command-match", "ansi-green");
  ic_style_def("command-no-match", "ansi-red");
  ic_style_def("file-exists", "underline");
  ic_style_def("quoted", "ansi-yellow");
  ic_style_def("quoted-file-exists", "quoted file-exists");
}

void Prompt::configure_completer() {
  ic_completer_fun_t* completer_func = [](ic_completion_env_t* cenv,
                                          const char* prefix) {
    Prompt* prompt = static_cast<Prompt*>(ic_completion_arg(cenv));
    prompt->completer(cenv, prefix);
  };

  ic_set_default_completer(completer_func, this);
}

void Prompt::configure_highlighter() {
  ic_highlight_fun_t* highlighter_func = [](ic_highlight_env_t* henv,
                                            const char* input, void* arg) {
    Prompt* prompt = static_cast<Prompt*>(arg);
    prompt->highlighter(henv, input);
  };

  ic_set_default_highlighter(highlighter_func, this);
}

void Prompt::enable_auto_completion() {
  // Try to auto-complete after a completion as long as the completion is
  // unique.
  ic_enable_auto_tab(true);
}

void Prompt::enable_inline_hints() {
  // Should be enabled by default but just in case.
  ic_enable_hint(true);
}

void Prompt::completer(ic_completion_env_t* cenv, const char* input) {
  ic_completer_fun_t* word_completer_func = [](ic_completion_env_t* cenv,
                                               const char* prefix) {
    Prompt* prompt = static_cast<Prompt*>(ic_completion_arg(cenv));
    prompt->word_completer(cenv, prefix);
  };

  ic_complete_word(cenv, input, word_completer_func, nullptr);

  // Complete file paths if the current command accepts them.

  long cursor;
  (void)ic_completion_input(cenv, &cursor);

  if (is_cursor_in_first_word(input, cursor))  // No command given yet
    return;

  CommandIterator it = match_current_command(input);
  if (it == m_commands.end())  // The command is unknown
    return;

  const auto& [name, info] = *it;
  if (info.prompt_info.can_accept_file_paths) {
    ic_complete_filename(cenv, input, 0, ".", nullptr);
  }
}

void Prompt::word_completer(ic_completion_env_t* cenv, const char* word) {
  long cursor;
  const char* input = ic_completion_input(cenv, &cursor);

  if (is_cursor_in_first_word(input, cursor)) {
    complete_command_name(cenv, word);
    return;
  }

  if (auto it = match_current_command(input); it != m_commands.end()) {
    if (!*word || *word == '-') {
      complete_command_options(cenv, it, word);
    }
    complete_command_non_options(cenv, it, word);
  }
}

void Prompt::complete_command_name(ic_completion_env_t* cenv,
                                   const char* word) {
  for (const auto& [name, info] : m_commands) {
    if (name.starts_with(word)) {
      (void)ic_add_completion(cenv, name.c_str());
    }
  }
}

void Prompt::complete_command_options(ic_completion_env_t* cenv,
                                      CommandIterator command_it,
                                      const char* word) {
  const auto& [name, info] = *command_it;
  const size_t word_len = strlen(word);

  bool double_dash = strncmp(word, "--", 2) == 0;

  for (const Option& option : info.prompt_info.options) {
    for (const char* name : option.name.names()) {
      if (name == nullptr)
        continue;

      size_t name_len = strlen(name);
      bool single_dash_option = name_len == 1;
      if (double_dash && single_dash_option)
        continue;

      const char* prefix = single_dash_option ? "-" : "--";
      const char* suffix = option.requires_value ? "=" : "";

      std::string option_name = std::string(prefix) + name + suffix;

      if (word_len >= option_name.length())
        continue;

      if (strncmp(option_name.c_str(), word, word_len) == 0) {
        ic_add_completion(cenv, option_name.c_str());
      }
    }
  }
}

void Prompt::complete_command_non_options(struct ic_completion_env_s* cenv,
                                          CommandIterator command_it,
                                          const char* word) {
  const auto& [name, info] = *command_it;
  const size_t word_len = strlen(word);

  const auto non_options = info.prompt_info.non_options;
  if (!non_options)
    return;

  for (const std::string& non_option : *non_options) {
    if (non_option.length() < word_len)
      continue;

    if (non_option.starts_with(word)) {
      ic_add_completion(cenv, non_option.c_str());
    }
  }
}

void Prompt::highlighter(ic_highlight_env_t* henv, const char* input_begin) {
  const char* input = input_begin;

  const char* word_begin;
  size_t word_len;
  bool is_quoted;
  bool first_word = true;
  while ((word_begin = get_next_word(&input, &word_len, &is_quoted))) {
    size_t word_begin_index = word_begin - input_begin;
    size_t word_end_index = word_begin_index + word_len;

    std::string_view word(word_begin, word_len);

    if (is_quoted) {  // Highlight the quotations as well as the contents
      word_begin_index--;
      word_end_index++;
    }

    if (first_word) {
      first_word = false;

      bool matches_command = false;
      for (const auto& [name, id] : m_commands) {
        if (name.length() != word_len)
          continue;
        if (word.compare(name) == 0) {
          matches_command = true;
          break;
        }
      }

      ic_highlight(henv, word_begin_index, word_end_index - word_begin_index,
                   matches_command ? "command-match" : "command-no-match");

    } else if (std::filesystem::exists(word)) {
      ic_highlight(henv, word_begin_index, word_end_index - word_begin_index,
                   is_quoted ? "quoted-file-exists" : "file-exists");

    } else if (is_quoted) {
      ic_highlight(henv, word_begin_index, word_end_index - word_begin_index,
                   "quoted");
    }
  }
}

Prompt::CommandInvocation Prompt::parse_command_invocation(
    const char* input_begin) {
  assert(input_begin != nullptr);

  CommandInvocation invocation;

  const char* input = input_begin;

  const char* word_begin;
  size_t word_len;
  bool first_word = true;
  while ((word_begin = get_next_word(&input, &word_len))) {
    size_t word_begin_index = word_begin - input_begin;
    size_t word_end_index = word_begin_index + word_len;

    std::string word(word_begin, word_len);

    if (first_word) {
      first_word = false;
      if (auto it = m_commands.find(word); it != m_commands.end()) {
        invocation.command = it;
      }
    }

    invocation.args.push_back(std::move(word));
  }

  return invocation;
}

Prompt::CommandIterator Prompt::match_current_command(const char* input) {
  size_t first_word_len;
  const char* first_word = get_first_word(input, &first_word_len);
  if (!first_word)
    return m_commands.end();

  return m_commands.find(std::string(first_word, first_word_len));
}

const char* Prompt::get_next_word(const char** input,
                                  size_t* word_len,
                                  bool* is_quoted) {
  assert(input != nullptr && *input != nullptr);
  assert(word_len != nullptr);

  // Skip leading whitespace
  const char* word_begin = *input;
  for (; *word_begin && isspace(*word_begin); ++word_begin)
    ;

  if (!*word_begin)  // No word found
    return nullptr;

  // Check for beginning of quote
  char quote = 0;
  switch (*word_begin) {
    case '\'':
    case '"':
      quote = *word_begin;
      break;
  }
  if (is_quoted)
    *is_quoted = (quote != 0);

  // Skip over word
  const char* word_end = word_begin + 1;
  char c;
  bool end_quote = false;
  while ((c = *word_end)) {
    if (quote) {
      // Don't stop until end quote
      if (c == quote) {
        end_quote = true;
        break;
      }
    } else {
      // Don't stop until space or quote
      if (isspace(c) || c == '\'' || c == '"')
        break;
    }
    ++word_end;
  }

  if (quote)
    ++word_begin;  // Don't include starting quote

  *word_len = word_end - word_begin;

  *input = word_begin + *word_len + end_quote;  // Skip end quote too
  return word_begin;
}

bool Prompt::is_cursor_in_first_word(const char* input, long cursor) {
  size_t word_len;
  const char* word_begin = get_first_word(input, &word_len);

  if (!word_begin)
    return true;

  const char* word_end = word_begin + word_len;
  const char* cursor_pos = input + cursor;
  return cursor_pos >= word_begin && cursor_pos <= word_end;
}
