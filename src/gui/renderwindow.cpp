// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "renderwindow.h"

#include "../core/workspace.h"
#include "../render/framebuffer.h"
#include "../render/renderer.h"
#include "../render/resource.h"
#include "../render/shader.h"
#include "../render/texture.h"
#include "../render/uniformbuffer.h"

#include <QOpenGLPaintDevice>

RenderWindowBase::RenderWindowBase() {
  LockScope ws;
  class RenderWidget : public QOpenGLWidget {
    RenderWindowBase *_parent = nullptr;

  public:
    RenderWidget(RenderWindowBase *parent)
        : QOpenGLWidget(parent), _parent(parent) {}
    void initializeGL() override {}
    virtual bool event(QEvent *e) override {
      if (e->type() == QEvent::Show) {
        LOG_DEBUG("show event");
        GlobalEvents::instance()->redraw();
      }
      bool ret = QWidget::event(e);
      _parent->handleEvent(e);
      return ret;
    }
    void resizeGL(int w, int h) override {
      LOG_DEBUG("resize");
      _parent->_width = w;
      _parent->_height = h;
      GlobalEvents::instance()->redraw();
    }
    void paintGL() override {
      V_GL(glClearColor(0, 0, 0, 1));
      V_GL(glClear(GL_COLOR_BUFFER_BIT));
      if (int target = defaultFramebufferObject()) {
        //_parent->_render_target.present(target);
        _parent->composite(target);
      }
      {
        QPainter painter(this);
        painter.endNativePainting();
        _parent->paintHUD(&painter);
        painter.beginNativePainting();
      }
    }
  };
  setContentWidget(new RenderWidget(this));
}

RenderWindowBase::~RenderWindowBase() {}

void RenderWindowBase::present() { contentWidget()->update(); }

void RenderWindowBase::renderWindowSync(
    const RenderWindowSyncContext &context) {}

void RenderWindowBase::renderWindowAsync(
    const RenderWindowAsyncContext &context) {}
