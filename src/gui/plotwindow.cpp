// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "plotwindow.h"

#include "../core/bagplayer.h"
#include "../core/log.h"
#include "../core/topic.h"
#include "../core/workspace.h"

#include <QGraphicsScene>
#include <QGraphicsView>

PlotWindow::PlotWindow() {

  LockScope()->modified.connect(this, [this]() {
    LockScope ws;
    auto plot_display = plot().resolve(ws.ws());
    if (!plot_display) {
      _renderer.reset();
    } else {
      if (!_renderer || plot_display != _renderer->plotDisplay()) {
        LOG_DEBUG("creating plot renderer");
        _renderer = std::make_shared<PlotRenderer>(plot_display);
      }
    }
    GlobalEvents::instance()->redraw();
  });

  {
    auto *button = new FlatButton();
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    QMenu *menu = new QMenu(this);
    button->setMenu(menu);
    button->setText("Select plot...");
    addToolWidget(button);
    connect(menu, &QMenu::aboutToShow, this, [this, menu]() {
      menu->clear();
      LockScope()->document()->display()->recurse(
          [&](const std::shared_ptr<Display> &display) {
            if (auto plot = std::dynamic_pointer_cast<PlotDisplay>(display)) {
              connect(menu->addAction(plot->name().c_str()),
                      &QAction::triggered, this, [this, plot](bool) {
                        ActionScope ws("Plot");
                        this->plot() = plot;
                        LockScope()->selection() = plot;
                        ws->modified();
                      });
            }
          });
      connect(menu->addAction("Create new plot"), &QAction::triggered, this,
              [this](bool) {
                ActionScope ws("Plot");
                auto plot = std::make_shared<PlotDisplay>();
                static size_t counter = 1;
                plot->name() = "Plot " + std::to_string(counter++);
                LockScope()->document()->display()->displays().push_back(plot);
                LockScope()->selection() = plot;
                this->plot() = plot;
                ws->modified();
              });
    });
    LockScope ws;
    ws->modified.connect(button, [this, button]() {
      LockScope ws;
      if (auto plot = this->plot().resolve(ws.ws())) {
        button->setText(plot->name().c_str());
      } else {
        button->setText("Select plot...");
      }
    });
  }

  {
    auto *button = new FlatButton();
    button->setIcon(MATERIAL_ICON("multiline_chart"));
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(button, &QPushButton::clicked, this, [this]() {
      LockScope ws;
      ws->selection() = plot().resolve(ws->document());
      ws->modified();
    });
    addToolWidgetRight(button);
  };
}

void PlotWindow::renderWindowSync(const RenderWindowSyncContext &context) {
  // LOG_DEBUG(this);
  _renderer_async = _renderer;
  if (_renderer) {
    _renderer->renderSync();
  } else {
    LOG_DEBUG("waiting for plot renderer sync");
  }
  if (!LockScope()->player) {
    GlobalEvents::instance()->redraw();
  }
}

void PlotWindow::renderWindowAsync(const RenderWindowAsyncContext &context) {
  // LOG_DEBUG(this);
#if 1
  auto *previous_context = QOpenGLContext::currentContext();
  auto *previous_surface = previous_context->surface();

  if (!_offscreen_context) {
    _offscreen_context = std::make_shared<QOpenGLContext>();
    _offscreen_context->setShareContext(QOpenGLContext::globalShareContext());
    if (!_offscreen_context->create()) {
      throw std::runtime_error("failed to create opengl context");
    }
  }

  if (!_offscreen_surface) {
    _offscreen_surface = std::make_shared<QOffscreenSurface>();
    _offscreen_surface->create();
    if (!_offscreen_surface->isValid()) {
      throw std::runtime_error("failed to create offscreen surface");
    }
  }

  _offscreen_context->makeCurrent(_offscreen_surface.get());

  if (!_fbo_render || _fbo_render->width() != _width ||
      _fbo_render->height() != _height) {
    QOpenGLFramebufferObjectFormat format;
    // format.setSamples(0);
    // format.setSamples(16);
    format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
    _fbo_render =
        std::make_shared<QOpenGLFramebufferObject>(_width, _height, format);
  }

  if (!_paint_device) {
    _paint_device =
        std::make_shared<QOpenGLPaintDevice>(QSize(_width, _height));
  }
  _paint_device->setSize(QSize(_width, _height));

  if (!_fbo_render->bind()) {
    throw std::runtime_error("failed to bind fbo");
  }

  {

    QPainter painter(_paint_device.get());

    // painter.endNativePainting();
    painter.beginNativePainting();
    V_GL(glClearColor(1, 0, 1, 1));
    V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |
                 GL_STENCIL_BUFFER_BIT));
    painter.endNativePainting();

    // painter.setRenderHint(QPainter::Antialiasing);

    if (_renderer_async) {
      _renderer_async->renderAsync(&painter);
    } else {
      LOG_DEBUG("waiting for plot renderer async");
    }

    // painter.endNativePainting();
    // painter.beginNativePainting();
  }

  _fbo_render->release();

  {
    std::unique_lock<std::mutex> lock(_fbo_mutex);

    _fbo_buffer.update(_width, _height, GL_RGB8);

    _fbo_present.bind();
    _fbo_present.attach(_fbo_buffer, GL_COLOR_ATTACHMENT0);

    V_GL(glBindFramebuffer(GL_READ_FRAMEBUFFER, _fbo_render->handle()));
    V_GL(glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _fbo_present.id()));
    V_GL(glBlitFramebuffer(0, 0, _width, _height, 0, 0, _width, _height,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST));

    _fbo_present.bind();
    V_GL(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_RENDERBUFFER, 0));

    V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));

    _fbo_initialized = true;
  }

  previous_context->makeCurrent(previous_surface);
#endif
}

void PlotWindow::paintHUD(QPainter *p) {
#if 0
  /*
    if (_renderer_async) {
      _renderer_async->renderAsync(p);
    }
    */
  LockScope ws;
  if (_renderer) {
    _renderer->renderAsync(p);
  }
  /*
  p->beginNativePainting();
  V_GL(glFlush());
  V_GL(glFinish());
  V_GL(glFlush());
  p->endNativePainting();
  */
#endif
}

void PlotWindow::composite(int target) {
  // LOG_DEBUG(this);
#if 1
  std::unique_lock<std::mutex> lock(_fbo_mutex);

  V_GL(glViewport(0, 0, _width, _height));
  V_GL(glClearColor(0, 1, 0, 1));
  V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

  if (_fbo_initialized) {
    _fbo_composite.bind();
    _fbo_composite.attach(_fbo_buffer, GL_COLOR_ATTACHMENT0);

    V_GL(glBindFramebuffer(GL_READ_FRAMEBUFFER, _fbo_composite.id()));
    V_GL(glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target));
    V_GL(glBlitFramebuffer(0, 0, _width, _height, 0, 0, _width, _height,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST));

    _fbo_composite.bind();
    V_GL(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_RENDERBUFFER, 0));

    V_GL(glBindFramebuffer(GL_FRAMEBUFFER, 0));
  } else {
    LOG_DEBUG("waiting for fbo " << this);
  }
#endif
}

void PlotWindow::handleEvent(QEvent *event) {
  switch (event->type()) {
  case QEvent::MouseButtonPress:
    if (_renderer) {
      ActionScope ws("Pick");
      ws->selection() = _renderer->plotDisplay();
    }
    break;
  }
}
