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
      V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
      if (int target = defaultFramebufferObject()) {
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
  auto *render_widget = new RenderWidget(this);
  setContentWidget(render_widget);

  {
    auto *button = new FlatButton();
    button->setIcon(MATERIAL_ICON("photo_camera", 0.0));
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(button, &QPushButton::clicked, this, [this, render_widget]() {
      QImage screenshot;
      {
        render_widget->makeCurrent();
        {
          QOpenGLFramebufferObject fbo(render_widget->width(),
                                       render_widget->height());
          composite(fbo.handle());
          screenshot = fbo.toImage();
        }
        render_widget->doneCurrent();
      }
      QString file_name = QFileDialog::getSaveFileName(
          this, tr("Save Screenshot"), "", tr("Images (*.png *.jpg)"));
      if (!file_name.isEmpty()) {
        if (QFileInfo(file_name).suffix().isEmpty()) {
          if (!file_name.endsWith(".")) {
            file_name += ".";
          }
          file_name += "png";
        }
        if (screenshot.save(file_name)) {
          LOG_SUCCESS("screenshot saved: " << file_name.toStdString());
        } else {
          LOG_ERROR("failed to save screenshot: " << file_name.toStdString());
        }
      }
    });
    addToolWidgetRight(button);
  }
}

RenderWindowBase::~RenderWindowBase() {}

void RenderWindowBase::present() { contentWidget()->update(); }

void RenderWindowBase::renderWindowSync(
    const RenderWindowSyncContext &context) {}

void RenderWindowBase::renderWindowAsync(
    const RenderWindowAsyncContext &context) {}
