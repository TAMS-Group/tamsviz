// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <iostream>
#include <sstream>

#include "event.h"

enum class LogLevel {
  Debug,
  Info,
  Warn,
  Error,
  Fatal,
  Success,
};

class LogMessage {
  LogLevel _level;
  const char *_message;
  const char *_file;
  size_t _line;
  const char *_function;
  const char *_node;

public:
  LogMessage(const LogLevel &level, const char *message, const char *file,
             size_t line, const char *function, const char *node)
      : _level(level), _message(message), _file(file), _line(line),
        _function(function), _node(node) {}
  LogMessage(const LogMessage &) = delete;
  LogMessage &operator=(const LogMessage &) = delete;
  LogLevel level() const { return _level; }
  const char *message() const { return _message; }
  const char *file() const { return _file; }
  size_t line() const { return _line; }
  const char *function() const { return _function; }
  const char *node() const { return _node; }
};

struct LogManager {
  LogManager();
  Event<void(const LogMessage &)> received;
  static LogManager &instance();
};

#define LOG_IMPL(level, condition, ...)                                        \
  if (condition) {                                                             \
    std::ostringstream log_ostream_internal; /* TODO: replace */               \
    log_ostream_internal << __VA_ARGS__;                                       \
    LogManager::instance().received(                                           \
        LogMessage(level, log_ostream_internal.str().c_str(), __FILE__,        \
                   __LINE__, __PRETTY_FUNCTION__, ""));                        \
  }

#define LOG_THROTTLE_IMPL(interval)                                            \
  ([]() {                                                                      \
    static std::chrono::time_point<std::chrono::steady_clock> l;               \
    auto t = std::chrono::steady_clock::now();                                 \
    if (t - l > std::chrono::duration<double>(interval)) {                     \
      l = t;                                                                   \
      return true;                                                             \
    } else {                                                                   \
      return false;                                                            \
    }                                                                          \
  }())

#define LOG_DEBUG(...) LOG_IMPL(LogLevel::Debug, 1, __VA_ARGS__)
#define LOG_INFO(...) LOG_IMPL(LogLevel::Info, 1, __VA_ARGS__)
#define LOG_WARN(...) LOG_IMPL(LogLevel::Warn, 1, __VA_ARGS__)
#define LOG_ERROR(...) LOG_IMPL(LogLevel::Error, 1, __VA_ARGS__)
#define LOG_FATAL(...) LOG_IMPL(LogLevel::Fatal, 1, __VA_ARGS__)
#define LOG_SUCCESS(...) LOG_IMPL(LogLevel::Success, 1, __VA_ARGS__)

#define LOG_DEBUG_THROTTLE(interval, ...)                                      \
  LOG_IMPL(LogLevel::Debug, LOG_THROTTLE_IMPL(interval), __VA_ARGS__)
#define LOG_INFO_THROTTLE(interval, ...)                                       \
  LOG_IMPL(LogLevel::Info, LOG_THROTTLE_IMPL(interval), __VA_ARGS__)
#define LOG_WARN_THROTTLE(interval, ...)                                       \
  LOG_IMPL(LogLevel::Warn, LOG_THROTTLE_IMPL(interval), __VA_ARGS__)
#define LOG_ERROR_THROTTLE(interval, ...)                                      \
  LOG_IMPL(LogLevel::Error, LOG_THROTTLE_IMPL(interval), __VA_ARGS__)
#define LOG_FATAL_THROTTLE(interval, ...)                                      \
  LOG_IMPL(LogLevel::Fatal, LOG_THROTTLE_IMPL(interval), __VA_ARGS__)
#define LOG_SUCCESS_THROTTLE(interval, ...)                                    \
  LOG_IMPL(LogLevel::Success, LOG_THROTTLE_IMPL(interval), __VA_ARGS__)
