#include <micromouse_cli/commands/command.hpp>

#include <cassert>
#include <cstdio>
#include <format>

#define INDENT 4
#define OPTION_HELP_TEXT_COLUMN 20

void Command::help(const char* command_name,
                   const char* usage_text,
                   const char* description_text,
                   std::span<const Option> options) {
  assert(command_name != nullptr);

  printf("Usage: %s\n\n", usage_text ? usage_text : command_name);
  if (description_text) {
    printf("%s\n", description_text);
  }

  if (options.empty())
    return;

  puts("\nOptions:");

  for (const Option& option : options) {
    const char* primary_name = option.name.primary();
    if (primary_name == nullptr)
      continue;

    size_t name_len = strlen(primary_name);

    std::string prefix = name_len == 1 ? "-" : "--";

    std::string suffix;
    if (option.requires_value) {
      const char* value_text = option.value_text;
      if (!value_text)
        value_text = "value";

      suffix = std::format("=<{}>", value_text);
    }

    std::string text = prefix + primary_name + suffix;

    // Indent
    printf("%*s", INDENT, "");
    printf("%s", text.c_str());

    if (!option.help) {
      putchar('\n');
      continue;
    }

    int remaining_space =
        OPTION_HELP_TEXT_COLUMN - static_cast<int>(text.length()) - INDENT;

    if (remaining_space > 0) {
      printf("%*s", remaining_space, "");
    } else {
      // If the text is too long, print the text on the next line
      printf("\n%*s", OPTION_HELP_TEXT_COLUMN, "");
    }

    // TODO: Wrap?
    printf("%s\n", option.help);
  }
}
