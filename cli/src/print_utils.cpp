#include <micromouse_cli/print_utils.hpp>

#include <cassert>

void print_line_wrapped(FILE* stream,
                        std::string_view text,
                        size_t wrap_width,
                        size_t indent,
                        size_t current_indent) {
  assert(stream != nullptr);
  assert(indent < wrap_width);
  assert(current_indent < wrap_width);

  while (!text.empty()) {
    // Print indent
    if (current_indent < indent) {
      (void)fprintf(stream, "%*s", (int)(indent - current_indent), "");
    }

    size_t text_len = text.length();

    // No need to wrap
    if (text_len <= wrap_width - indent) {
      (void)fprintf(stream, "%.*s\n", (int)text_len, text.data());
      return;
    }

    size_t line_len = wrap_width - indent;

    bool replace_space;

    // Find the last space in the line
    size_t last_space = text.rfind(' ', line_len);
    replace_space = last_space != std::string_view::npos;
    if (replace_space) {
      line_len = last_space;
    }

    // Print up to the last space
    (void)fprintf(stream, "%.*s\n", (int)line_len, text.data());

    // Skip the printed text
    text.remove_prefix(line_len + replace_space);

    current_indent = 0;
  }
}

void print_wrapped_named_field(FILE* stream,
                               std::string_view name,
                               std::string_view value,
                               size_t indent,
                               size_t field_name_column_width,
                               size_t wrap_width) {
  assert(stream != nullptr);
  assert(indent < field_name_column_width);
  assert(field_name_column_width < wrap_width);

  // Indent
  (void)fprintf(stream, "%*s", (int)indent, "");
  (void)fprintf(stream, "%.*s", (int)name.length(), name.data());

  if (value.empty()) {
    (void)fputc('\n', stream);
    return;
  }

  int remaining_space =
      static_cast<int>(field_name_column_width) - name.length() - indent;

  if (remaining_space > 0) {
    (void)fprintf(stream, "%*s", remaining_space, "");
  } else {
    // If the text is too long, print the text on the next line
    (void)fprintf(stream, "\n%*s", (int)field_name_column_width, "");
  }

  print_line_wrapped(stream, value, wrap_width, field_name_column_width,
                     field_name_column_width);
}
