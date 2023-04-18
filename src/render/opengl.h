// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <stdexcept>

void tamsviz_gl_begin(const char *file, int line, const char *cmd);
void tamsviz_gl_end(const char *file, int line, const char *cmd);

#define V_GL(x)                                                                \
  tamsviz_gl_begin(__FILE__, __LINE__, #x);                                    \
  x;                                                                           \
  tamsviz_gl_end(__FILE__, __LINE__, #x);
