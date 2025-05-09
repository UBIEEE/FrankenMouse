#include <micromouse_cli/diagnostics.hpp>

#include <cassert>
#include <cstdarg>
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

  report(stderr, tag,
         "\033[33;1m"
         "warning: "
         "\033[m",
         fmt, args);

  va_end(args);
}

void report_error(const char* tag, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);

  report(stderr, tag,
         "\033[31;1m"
         "error: "
         "\033[m",
         fmt, args);

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

  (void)fputs("\r\033[K\033[m", stream);  // Clear line, reset style

  if (tag) {
    (void)fprintf(stream, "\033[1m%s\033[1;m: \033[m", tag);
  }

  if (prefix) {
    (void)fprintf(stream, "%s", prefix);
  }

  (void)vfprintf(stream, fmt, args);
  (void)fputc('\n', stream);
}
