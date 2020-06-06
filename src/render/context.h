// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include <functional>

#include "../core/destructor.h"

class GLBase {
public:
  GLBase() {}
  GLBase(const GLBase &) = delete;
  GLBase &operator=(const GLBase &) = delete;
};

class GLDisplay : public GLBase {
  Destructor destructor;
  void *egl_display = nullptr;

public:
  GLDisplay();
  void *eglDisplay() { return egl_display; }
  friend class GLContext;
  friend class GLSurface;
  friend class GLScope;
};

class GLContext : public GLBase {
  GLDisplay *display = nullptr;
  void *egl_context = nullptr;

public:
  GLContext(GLDisplay *display);
  ~GLContext();
  void *eglContext() { return egl_context; }
  friend class GLScope;
  friend class GLSurface;
  friend class GLScope;
};

class GLSurface : public GLBase {
  GLDisplay *display = nullptr;
  void *egl_surface = nullptr;

public:
  GLSurface(GLDisplay *display);
  GLSurface(GLDisplay *display, long long window);
  ~GLSurface();
  void swap();
  // int width() const;
  // int height() const;
  friend class GLScope;
};

class GLScope : public GLBase {
  GLDisplay *display = nullptr;

public:
  GLScope(GLContext *context, GLSurface *surface);
  ~GLScope();
};
