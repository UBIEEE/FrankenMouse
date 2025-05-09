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
