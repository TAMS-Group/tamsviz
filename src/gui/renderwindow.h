// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "renderthread.h"
#include "splitwindow.h"

#include "../render/rendertarget.h"

class UniformBufferBase;
class RenderList;
class Renderer;
class CameraBlock;
class Framebuffer;
class Texture;

struct RenderWindowSyncContext {};

struct RenderWindowAsyncContext {
  RenderList *render_list = nullptr;
  Renderer *renderer = nullptr;
};

class RenderWindowBase : public ContentWindowBase {

protected:
  volatile int _width = 1, _height = 1;

protected:
  RenderWindowBase();
  ~RenderWindowBase();

public:
  virtual void composite(int target) {}
  virtual void renderWindowSync(const RenderWindowSyncContext &context);
  virtual void renderWindowAsync(const RenderWindowAsyncContext &context);
  virtual void handleEvent(QEvent *event) {}
  int renderWidth() const { return _width; }
  int renderHeight() const { return _height; }
  void present();
  virtual void paintHUD(QPainter *painter) {}
};
DECLARE_TYPE(RenderWindowBase, ContentWindowBase);
