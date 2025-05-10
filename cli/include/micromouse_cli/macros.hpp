#pragma once

#if defined(__MINGW32__) && !defined(__clang__)
#define MM_FMTARGS(FMT) __attribute__((format(gnu_printf, FMT, FMT + 1)))
#define MM_FMTLIST(FMT) __attribute__((format(gnu_printf, FMT, 0)))
#elif defined(__clang__) || defined(__GNUC__)
#define MM_FMTARGS(FMT) __attribute__((format(printf, FMT, FMT + 1)))
#define MM_FMTLIST(FMT) __attribute__((format(printf, FMT, 0)))
#else
#define MM_FMTARGS(FMT)
#define MM_FMTLIST(FMT)
#endif

// ANSI escape codes for colors/styles

#define BOLD(str) "\033[1m" str "\033[m"

#define RED(str) "\033[31m" str "\033[m"
#define GREEN(str) "\033[32m" str "\033[m"
#define YELLOW(str) "\033[33m" str "\033[m"
#define BLUE(str) "\033[34m" str "\033[m"
#define MAGENTA(str) "\033[35m" str "\033[m"
#define CYAN(str) "\033[36m" str "\033[m"

// Cursor to column 1, clear line, reset style
#define CLEAR_LINE() "\r\033[K\033[m"

// Clear terminal, move cursor to 1,1, reset style
#define CLEAR_SCREEN() "\033[2J\033[;H\033[m"
