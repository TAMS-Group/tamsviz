// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <stdexcept>

#define V_GL(x)                                                                \
  while (glGetError()) {                                                       \
  }                                                                            \
  x;                                                                           \
  while (int e = glGetError()) {                                               \
    throw std::runtime_error(std::string(#x " failed with error #") +          \
                             std::to_string(e));                               \
  }
