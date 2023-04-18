// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "opengl.h"

#include "../core/log.h"

#define TAMSVIZ_GL_ENUM(x)                                                     \
  case x:                                                                      \
    return #x;

static std::string gl_error_to_string(int err) {
  switch (err) {
    TAMSVIZ_GL_ENUM(GL_INVALID_ENUM)
    TAMSVIZ_GL_ENUM(GL_INVALID_VALUE)
    TAMSVIZ_GL_ENUM(GL_INVALID_OPERATION)
    TAMSVIZ_GL_ENUM(GL_STACK_OVERFLOW)
    TAMSVIZ_GL_ENUM(GL_STACK_UNDERFLOW)
    TAMSVIZ_GL_ENUM(GL_OUT_OF_MEMORY)
    TAMSVIZ_GL_ENUM(GL_INVALID_FRAMEBUFFER_OPERATION)
    TAMSVIZ_GL_ENUM(GL_CONTEXT_LOST)
  default:
    return "???";
  }
}

static std::string gl_fbo_status_to_string(int err) {
  switch (err) {
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_COMPLETE)
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_UNDEFINED)
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT)
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT)
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER)
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER)
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_UNSUPPORTED)
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE)
    TAMSVIZ_GL_ENUM(GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS)
  default:
    return "???";
  }
}

void tamsviz_gl_begin(const char *file, int line, const char *cmd) {
  while (glGetError()) {
  }
}

void tamsviz_gl_end(const char *file, int line, const char *cmd) {

  while (int e = glGetError()) {

    LOG_ERROR(cmd << " failed with error " << e << " " << ((void *)intptr_t(e))
                  << " " << gl_error_to_string(e) << " @ " << file << " "
                  << line);

    if (e == GL_INVALID_FRAMEBUFFER_OPERATION) {
      int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
      LOG_ERROR("fbo status " << status << " " << ((void *)intptr_t(status))
                              << " " << gl_fbo_status_to_string(status));
    }

    throw std::runtime_error(std::string(cmd) + " failed with error #" +
                             std::to_string(e));
  }
}
