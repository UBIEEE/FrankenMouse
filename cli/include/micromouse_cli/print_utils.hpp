#pragma once

#include <cstdio>
#include <string_view>

#define DEFAULT_WRAP_WIDTH 80
#define DEFAULT_INDENT 4
#define DEFAULT_FIELD_NAME_COLUMN_WIDTH 20

/**
 * @brief Print a line of text wrapped to a specified width. The text will be
 *        indented by the specified number of spaces.
 *
 *        This function splits the text in between words at whitespace
 *        characters. Continuous words that are longer than wrap_width will not
 *        be split.
 *
 * @param stream         File stream to print to (stdout, stderr, etc.)
 * @param text           Text to print. This should be a single line of text.
 *                       Strings with newline characters will be printed as-is
 *                       (probably not what you want).
 * @param wrap_width     Width to wrap the text to. The default is 80
 *                       characters.
 * @param indent         Number of spaces to print before each line.
 * @param current_indent The current indentation level. When this is greater
 *                       than zero, the number of spaces used to indent the
 *                       first line will be adjusted.
 */
void print_line_wrapped(FILE* stream,
                        std::string_view text,
                        size_t wrap_width = DEFAULT_WRAP_WIDTH,
                        size_t indent = DEFAULT_INDENT,
                        size_t current_indent = 0);

/**
 * @brief Print a line of text with a left and right column. The left column is
 *        indented by DEFAULT_INDENT spaces and contains the field's name. The
 *        right column contains the field's value. If the total width of the
 *        left column exceeds field_name_column_width, the right column will be
 *        printed on the next line. If the right column exceeds wrap_width, it
 *        will be wrapped to the next line.
 *
 * @param stream File stream to print to (stdout, stderr, etc.)
 * @param name   Name of the field. This will be printed in the left column.
 * @param value  Value of the field. This will be printed in the right column.
 */
void print_wrapped_named_field(
    FILE* stream,
    std::string_view name,
    std::string_view value,
    size_t indent = DEFAULT_INDENT,
    size_t field_name_column_width = DEFAULT_FIELD_NAME_COLUMN_WIDTH,
    size_t wrap_width = DEFAULT_WRAP_WIDTH);
