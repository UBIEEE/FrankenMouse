#pragma once

#if WITH_LOGGING

#include <micromouse/macros.hpp>
#include <spdlog/spdlog.h>

#ifndef LOG_PREFIX
#define LOG_PREFIX ""
#endif

// PLEASE don't have curly braces in file paths
#define LocString() "\n\tFile: " __FILE__ " \n\tLine: " TOSTRING(__LINE__)

#define LogInfo(msg, ...) spdlog::info(LOG_PREFIX msg __VA_OPT__(, ) __VA_ARGS__)
#define LogInfoLoc(msg, ...) \
  spdlog::info(LOG_PREFIX LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#define LogWarn(msg, ...) spdlog::warn(LOG_PREFIX msg __VA_OPT__(, ) __VA_ARGS__)
#define LogWarnLoc(msg, ...) \
  spdlog::warn(LOG_PREFIX LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#define LogError(msg, ...) spdlog::error(LOG_PREFIX msg __VA_OPT__(, ) __VA_ARGS__)
#define LogErrorLoc(msg, ...) \
  spdlog::error(LOG_PREFIX LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#define LogCritical(msg, ...) spdlog::critical(LOG_PREFIX msg __VA_OPT__(, ) __VA_ARGS__)
#define LogCriticalLoc(msg, ...) \
  spdlog::critical(LOG_PREFIX LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#define LogDebug(msg, ...) spdlog::debug(LOG_PREFIX msg __VA_OPT__(, ) __VA_ARGS__)
#define LogDebugLoc(msg, ...) \
  spdlog::debug(LOG_PREFIX LocString() "\n\tMessage: " msg "\n" __VA_OPT__(, ) __VA_ARGS__)

#else

#define LogInfo(msg, ...)
#define LogInfoLoc(msg, ...)

#define LogWarn(msg, ...)
#define LogWarnLoc(msg, ...)

#define LogError(msg, ...)
#define LogErrorLoc(msg, ...)

#define LogCritical(msg, ...)
#define LogCriticalLoc(msg, ...)

#define LogDebug(msg, ...)
#define LogDebugLoc(msg, ...)

#endif
