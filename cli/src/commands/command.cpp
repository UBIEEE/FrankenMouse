#include <micromouse_cli/commands/command.hpp>

#include <micromouse_cli/print_utils.hpp>

#include <cassert>
#include <format>

void Command::help(const char* command_name,
                   const PromptInfo& info,
                   FILE* stream) {
  assert(command_name != nullptr);
  assert(stream != nullptr);

  if (info.custom_help_func) {
    info.custom_help_func(stream);
    return;
  }

  (void)fprintf(stream, "Usage: %s\n\n",
                info.usage_text ? info.usage_text : command_name);

  const char* description = info.long_description_text;
  if (!description)
    description = info.short_description_text;

  if (description) {
    print_line_wrapped(stream, description, DEFAULT_WRAP_WIDTH, 0);
  }

  if (!info.options.empty()) {
    (void)fputs("\nOptions:\n", stream);

    for (const Option& option : info.options) {
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
      std::string_view help = option.help ? option.help : "";

      print_wrapped_named_field(stream, text, help);
    }
  }

  if (info.non_options_title && !info.non_options.empty()) {
    (void)fprintf(stream, "\n%s:\n", info.non_options_title);
    for (const std::string& non_option : info.non_options) {
      print_wrapped_named_field(stream, non_option, "");
    }
  }

  if (info.supplemental_help_func) {
    (void)fputs("\n", stream);
    info.supplemental_help_func(stream);
  }
}
