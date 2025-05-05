#include <micromouse_cli/prompt.hpp>

#include <isocline.h>
#include <cassert>
#include <filesystem>

void Prompt::register_command(const char* name,
                              std::span<const char*> options,
                              bool can_accept_file_paths,
                              MakeCommandFunc make_command) {
  CommandInfo info{
      .options = options,
      .can_accept_file_paths = can_accept_file_paths,
      .make_command_func = make_command,
  };

  m_commands.emplace(std::string(name), std::move(info));
}

Command* Prompt::readline() {
REPEAT:
  char* input = ic_readline(m_prompt_text.c_str());
  if (!input)
    return nullptr;

  CommandInvocation result = parse_command_invocation(input);
  free(input);

  if (result.command == m_commands.end()) {
    fprintf(stderr, "%s: unknown command: '%s'\n", m_prompt_text.c_str(),
            result.args[0].c_str());
    goto REPEAT;
  }

  const auto& [name, info] = *result.command;
  return info.make_command_func(std::move(result.args));
}

std::optional<std::string> Prompt::get_history_filename() {
  const char* home_dir = getenv("HOME");
  if (!home_dir)
    return std::nullopt;

  return std::string(home_dir) + "/mm_history.txt";
}

void Prompt::configure() {
  enable_history();
  configure_colors();
  configure_completer();
  configure_highlighter();
  enable_auto_completion();
  enable_inline_hints();
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
  if (info.can_accept_file_paths) {
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
    complete_command_options(cenv, it, word);
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

  for (const char* option : info.options) {
    assert(option != nullptr);
    if (word_len >= strlen(option))
      continue;

    if (strncmp(option, word, word_len) == 0) {
      ic_add_completion(cenv, option);
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
