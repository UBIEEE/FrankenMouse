#include <micromouse_cli/diagnostics.hpp>

#include <cassert>
#include <cstdarg>
#include <micromouse_cli/macros.hpp>
#include <mutex>

static std::mutex s_mutex;

static void report(FILE* stream,
                   const char* tag,
                   const char* prefix,
                   const char* fmt,
                   va_list args) MM_FMTLIST(4);

void report_status(const char* tag, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);

  report(stdout, tag, nullptr, fmt, args);

  va_end(args);
}

void report_warning(const char* tag, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);

  report(stderr, tag, BOLD(YELLOW("warning: ")), fmt, args);

  va_end(args);
}

void report_error(const char* tag, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);

  report(stderr, tag, BOLD(RED("error: ")), fmt, args);

  va_end(args);
}

static void report(FILE* stream,
                   const char* tag,
                   const char* prefix,
                   const char* fmt,
                   va_list args) {
  if (!stream)
    stream = stdout;
  assert(fmt != nullptr);

  std::lock_guard<std::mutex> lock(s_mutex);

  (void)fputs(CLEAR_LINE(), stream);  // Clear line, reset style

  if (tag) {
    (void)fprintf(stream, BOLD("%s: "), tag);
  }

  if (prefix) {
    (void)fprintf(stream, "%s", prefix);
  }

  (void)vfprintf(stream, fmt, args);
  (void)fputc('\n', stream);
}
