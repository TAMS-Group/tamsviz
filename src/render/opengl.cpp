// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "opengl.h"

#include "../core/log.h"

void tamsviz_gl_begin(const char *file, int line, const char *cmd) {
  while (glGetError()) {
  }
}

void tamsviz_gl_end(const char *file, int line, const char *cmd) {
  while (int e = glGetError()) {
    LOG_ERROR(cmd << " failed with error #" << e << " @ " << file << " "
                  << line);
    throw std::runtime_error(std::string(cmd) + " failed with error #" +
                             std::to_string(e));
  }
}
