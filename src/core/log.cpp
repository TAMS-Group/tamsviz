// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "log.h"

#include <cstring>

#define RGB(r, g, b) "\033[38;2;" #r ";" #g ";" #b "m"
#define RESET "\e[0m"
#define NEWLINE "\n"

LogManager::LogManager() {
  received.connect([](const LogMessage &m) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    const char *file = m.file();
    if (auto *c = std::strrchr(file, '/')) {
      file = c + 1;
    }
    const char *color = RGB(255, 255, 255);
    switch (m.level()) {
    case LogLevel::Debug:
      color = RGB(220, 220, 220);
      break;
    case LogLevel::Info:
      color = RGB(255, 255, 255) "\e[1m";
      break;
    case LogLevel::Warn:
      color = RGB(255, 255, 0) "\e[1m";
      break;
    case LogLevel::Error:
      color = RGB(255, 50, 0) "\e[1m";
      break;
    case LogLevel::Fatal:
      color = RGB(255, 50, 0) "\e[1m";
      break;
    case LogLevel::Success:
      color = RGB(50, 255, 0) "\e[1m";
      break;
    }
    std::cout << color << m.message() << RESET << RGB(100, 100, 100) << " "
              << m.function() << " " << file << ":" << m.line() << " "
              << m.node() << RESET << NEWLINE;
  });
}

static LogManager g_log_manager_instance;
LogManager &LogManager::instance() { return g_log_manager_instance; }
