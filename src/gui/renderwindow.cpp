// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "renderwindow.h"

#include "../core/bagplayer.h"
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
    void initializeGL() override { V_GL(glClearColor(1, 1, 1, 1)); }
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
      V_GL(glViewport(0, 0, w, h));
      GlobalEvents::instance()->redraw();
    }
    void paintGL() override {
      // LOG_DEBUG(this);
      V_GL(glViewport(0, 0, _parent->_width, _parent->_height));
      V_GL(glClearColor(0, 0, 1, 1));
      V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT |
                   GL_DEPTH_BUFFER_BIT));
      if (int target = defaultFramebufferObject()) {
        _parent->composite(target);
      } else {
        LOG_ERROR("failed to get framebfufer");
      }
      {
        QPainter painter(this);
        // painter.endNativePainting();
        _parent->paintHUD(&painter);
        // painter.beginNativePainting();
      }
      /*
      V_GL(glFlush());
      V_GL(glFinish());
      */
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
      std::string namesuggest = "";
      /*
      {
        LockScope ws;
        if (ws->player) {
          name = ws->player->path();
        }
      }
      name += ".3d.png";
      */
      namesuggest +=
          "3d-" + std::to_string(ros::WallTime::now().toNSec()) + ".png";
      {
        LockScope ws;
        if (ws->player) {
          std::string bagname = ws->player->fileName();
          for (auto &c : bagname) {
            if (!std::isalnum(c)) {
              c = '_';
            }
          }
          namesuggest = QFileInfo(ws->player->path().c_str())
                            .absoluteDir()
                            .filePath((bagname + "-" + namesuggest).c_str())
                            .toStdString();
        }
      }
      QString file_name = QFileDialog::getSaveFileName(
          this, tr("Save Screenshot"), namesuggest.c_str(),
          tr("Images (*.png *.jpg)"));
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
          QMessageBox::warning(nullptr, "Error", "Failed to save screenshot.");
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
