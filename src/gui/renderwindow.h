// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "renderthread.h"
#include "splitwindow.h"

class UniformBufferBase;
class RenderList;
class Renderer;
class CameraBlock;

struct RenderWindowSyncContext {};

struct RenderWindowAsyncContext {
  RenderList *render_list = nullptr;
  Renderer *renderer = nullptr;
};

class RenderWindowBase : public ContentWindowBase {
  std::unique_ptr<GLSurface> gl_surface;

protected:
  int _width = 1, _height = 1;

protected:
  RenderWindowBase();
  ~RenderWindowBase();

public:
  virtual void renderWindowSync(const RenderWindowSyncContext &context);
  virtual void renderWindowAsync(const RenderWindowAsyncContext &context);
  virtual void handleEvent(QEvent *event) {}
  GLSurface *surface() { return gl_surface.get(); }
};
DECLARE_TYPE(RenderWindowBase, ContentWindowBase);
