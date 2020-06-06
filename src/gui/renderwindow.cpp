// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "renderwindow.h"

#include "../core/workspace.h"

#include "../render/renderer.h"
#include "../render/resource.h"
#include "../render/shader.h"
#include "../render/uniformbuffer.h"

RenderWindowBase::RenderWindowBase() {
  LockScope ws;
  class RenderWidget : public QWidget {
    RenderWindowBase *_parent = nullptr;

  public:
    RenderWidget(RenderWindowBase *parent) : QWidget(parent), _parent(parent) {
      setStyleSheet("background-color:#000;");
    }
    virtual bool event(QEvent *e) override {
      if (e->type() == QEvent::Paint) {
        {
          LockScope ws;
          ws->redraw();
        }
        return true;
      }
      bool ret = QWidget::event(e);
      _parent->handleEvent(e);
      return ret;
    }
    virtual void resizeEvent(QResizeEvent *event) override {
      QWidget::resizeEvent(event);
      _parent->_width = std::max(1, event->size().width());
      _parent->_height = std::max(1, event->size().height());
      LOG_DEBUG("render window size " << _parent->_width << " "
                                      << _parent->_height);
      {
        LockScope ws;
        ws->redraw();
      }
    }
    virtual void paintEvent(QPaintEvent *event) override {
      LockScope ws;
      ws->redraw();
    }
  };
  auto *render_widget = new RenderWidget(this);
  render_widget->setAttribute(Qt::WA_NativeWindow, true);
  render_widget->setAttribute(Qt::WA_DontCreateNativeAncestors, true);
  render_widget->setAttribute(Qt::WA_PaintOnScreen, false);
  setContentWidget(render_widget);
  auto win_id = render_widget->winId();
  LOG_DEBUG("render window id " << win_id);
  gl_surface.reset(new GLSurface(RenderThread::instance()->display(), win_id));
}

RenderWindowBase::~RenderWindowBase() { gl_surface.reset(); }

void RenderWindowBase::renderWindowSync(
    const RenderWindowSyncContext &context) {}

void RenderWindowBase::renderWindowAsync(
    const RenderWindowAsyncContext &context) {}
