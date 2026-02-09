#pragma once

#include <micromouse_cli/macros.hpp>

// Thread-safe interface for reporting status, warning, and error messages.

/**
 * Emit a status message to stdout.
 */
void report_status(const char* tag, const char* fmt, ...) MM_FMTARGS(2);

/**
 * Emit a warning message to stderr.
 */
void report_warning(const char* tag, const char* fmt, ...) MM_FMTARGS(2);

/**
 * Emit an error message to stderr.
 */
void report_error(const char* tag, const char* fmt, ...) MM_FMTARGS(2);
