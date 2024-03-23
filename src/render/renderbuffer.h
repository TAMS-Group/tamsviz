// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "../core/watcher.h"

#include "opengl.h"
#include "resource.h"

class Renderbuffer : public ResourceBase {
protected:
  GLuint _id = 0;
  void destroy();
  Watcher _watcher;

public:
  ~Renderbuffer() { destroy(); }
  inline GLuint id() const { return _id; }
  void create();
  void update(int width, int height, int format, int samples = 0);
  void bind();
};
