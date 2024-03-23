// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "../render/opengl.h"

#include "renderwindow.h"

#include "../core/topic.h"
#include "../displays/plot.h"
#include "../render/renderer.h"
#include "../render/texture.h"

class QOpenGLPaintDevice;

class PlotWindow : public RenderWindowBase {
  std::shared_ptr<QOpenGLContext> _offscreen_context;
  std::shared_ptr<QOffscreenSurface> _offscreen_surface;
  std::shared_ptr<PlotRenderer> _renderer, _renderer_async;
  std::shared_ptr<QOpenGLPaintDevice> _paint_device;
  std::shared_ptr<QOpenGLFramebufferObject> _fbo_render;
  std::mutex _fbo_mutex;
  Framebuffer _fbo_composite;
  Framebuffer _fbo_present;
  Renderbuffer _fbo_buffer;
  bool _fbo_initialized = false;

public:
  PlotWindow();
  PROPERTY(Handle<PlotDisplay>, plot);
  virtual void
  renderWindowSync(const RenderWindowSyncContext &context) override;
  virtual void
  renderWindowAsync(const RenderWindowAsyncContext &context) override;
  virtual void paintHUD(QPainter *painter) override;
  virtual void composite(int target) override;
  virtual void handleEvent(QEvent *event) override;
};
DECLARE_TYPE(PlotWindow, RenderWindowBase);
