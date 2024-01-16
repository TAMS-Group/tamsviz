// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "scenewindow.h"

#include "../annotations/scene.h"
#include "../components/environment.h"
#include "../components/rendering.h"
#include "../core/bagplayer.h"
#include "../core/topic.h"
#include "../core/workspace.h"
#include "../displays/mesh.h"
#include "../render/resource.h"
#include "../render/shader.h"
#include "../render/transformations.h"
#include "../render/uniformbuffer.h"

SceneWindow::SceneWindow()
    : _uniform_buffer(
          new UniformBuffer<CameraBlock>((size_t)UniformBindingPoint::camera)) {
  {
    auto *button = new FlatButton();
    button->setText("Annotate");
    button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    QMenu *menu = new QMenu(this);
    button->setMenu(menu);
    addToolWidget(button);
    for (auto &type : Type::find<SceneAnnotationBase>()->list()) {
      if (!type->constructable()) {
        continue;
      }
      QString label = type->name().c_str();
      label = label.replace("SceneAnnotation", "");
      connect(
          menu->addAction(label), &QAction::triggered, this,
          [type, label, button, this](bool checked) {
            ActionScope ws("Annotate");
            if (!ws->player) {
              return;
            }
            auto timeline = ws->document()->timeline();
            if (!timeline) {
              return;
            }
            double current_time = ws->player->time();
            std::shared_ptr<AnnotationTrack> current_track =
                ws->currentAnnotationTrack().resolve(ws());
            if (!current_track) {
              for (auto &track_base : timeline->tracks()) {
                if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
                        track_base)) {
                  if (auto branch = track->branch(ws(), false)) {
                    for (auto &span : branch->spans()) {
                      if (span->start() <= current_time &&
                          span->start() + span->duration() >= current_time) {
                        current_track = track;
                        break;
                      }
                    }
                    if (current_track) {
                      break;
                    }
                  }
                }
              }
            }
            if (!current_track) {
              for (auto &track_base : timeline->tracks()) {
                if (auto track = std::dynamic_pointer_cast<AnnotationTrack>(
                        track_base)) {
                  current_track = track;
                  break;
                }
              }
            }
            if (!current_track) {
              timeline->tracks().push_back(
                  current_track = std::make_shared<AnnotationTrack>());
            }
            ws->currentAnnotationTrack() = current_track;
            std::shared_ptr<AnnotationSpan> current_span;
            if (auto branch = current_track->branch(ws(), false)) {
              for (auto &span : branch->spans()) {
                if (span->start() <= current_time &&
                    span->start() + span->duration() >= current_time) {
                  current_span = span;
                  break;
                }
              }
            }
            if (current_span == nullptr) {
              current_span = std::make_shared<AnnotationSpan>();
              current_span->start() = current_time;
              current_span->duration() = 0.1;
              current_track->branch(ws(), true)
                  ->spans()
                  .push_back(current_span);
            }
            auto annotation = type->instantiate<SceneAnnotationBase>();
            current_span->annotations().push_back(annotation);
            ws->modified();
          });
    }
  }
}

void SceneWindow::updateViewMatrix() {
  LockScope ws;
  _view_matrix =
      lookatMatrix(viewPosition(), viewTarget(), Eigen::Vector3d::UnitZ());
  _camera_block.view_matrix = _view_matrix.cast<float>();
}

void SceneWindow::renderWindowSync(const RenderWindowSyncContext &context) {
  LockScope ws;

  if (auto env = std::dynamic_pointer_cast<EnvironmentComponent>(
          ws->document()->display()->environment())) {
    _bgcolor = env->backgroundColor().toLinearVector4f() *
               float(env->backgroundBrightness());
  } else {
    LOG_ERROR_THROTTLE(1, "document broken, no environment component");
    _bgcolor.setZero();
  }

  updateViewMatrix();
  float far = (100.0 + (viewPosition() - viewTarget()).norm() * 2.0);
  float near = far * 0.0001f;
  _projection_matrix = projectionMatrix(fieldOfView() * M_PI / 180,
                                        _height * 1.0 / _width, near, far);
  _camera_block.projection_matrix = _projection_matrix.cast<float>();

  _camera_block.flags = 0;

  if (auto rendering_component = std::dynamic_pointer_cast<RenderingComponent>(
          ws->document()->display()->rendering())) {
    _multi_sampling = rendering_component->multiSampling();

    switch (rendering_component->sampleShading()) {
      case 1:
        _camera_block.flags |= CameraBlock::SampleShadingFlag;
        break;
      case 2:
        _camera_block.flags |= CameraBlock::SampleShadingFlag;
        _camera_block.flags |= CameraBlock::TransparentSampleShadingFlag;
        break;
    }
  } else {
    // LOG_ERROR_THROTTLE(1, "document broken, no rendering component");

    LOG_WARN("document broken, no rendering component, creating a new one");
    ws->document()->display()->rendering() =
        std::make_shared<RenderingComponent>();
    ws->modified();
  }
}

void SceneWindow::renderWindowAsync(const RenderWindowAsyncContext &context) {
  renderTarget().update(_width, _height, _multi_sampling);
  renderTarget().bind();
  V_GL(glViewport(0, 0, _width, _height));
  V_GL(glClearColor(_bgcolor.x(), _bgcolor.y(), _bgcolor.z(), _bgcolor.w()));
  V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
  if (!_action_queue.empty()) {
    std::lock_guard<std::mutex> lock(_action_mutex);
    for (auto &a : _action_queue) {
      a(context);
    }
    _action_tag.clear();
    _action_queue.clear();
    updateViewMatrix();
  }
  context.renderer->render(renderTarget(), _camera_block, *context.render_list);
}

void SceneWindow::composite(int target) { renderTarget().present(target); }

void SceneWindow::pushAction(
    const std::function<void(const RenderWindowAsyncContext &)> &action,
    const std::string &tag) {
  std::lock_guard<std::mutex> lock(_action_mutex);
  if (!tag.empty() && _action_tag == tag) {
    _action_queue.pop_back();
  }
  _action_tag = tag;
  _action_queue.push_back(action);
}

void SceneWindow::handleEvent(QEvent *event) {
  switch (event->type()) {
    case QEvent::Wheel: {
      LockScope ws;
      auto *wheel = static_cast<QWheelEvent *>(event);
      double degrees = wheel->angleDelta().y() * (1.0 / 8);
      double exponent = degrees / 90;
      double factor = std::pow(0.5, exponent);
      viewPosition() = (viewPosition() - viewTarget()) * factor + viewTarget();
      ws->modified();
      GlobalEvents::instance()->redraw();
      break;
    }
    case QEvent::MouseButtonPress: {
      LOG_DEBUG("press");
      auto *mouse = static_cast<QMouseEvent *>(event);
      {
        LockScope ws;
        _mouse_position = mouse->pos();
        _mouse_buttons = mouse->buttons();
        GlobalEvents::instance()->redraw();
      }
      if (mouse->button() == Qt::LeftButton) {
        _left_dragged = false;
        int x = mouse->x();
        int y = mouse->y();
        pushAction([this, x, y](const RenderWindowAsyncContext &context) {
          auto pick_result = context.renderer->pick(
              renderTarget(), _camera_block, *context.render_list, x,
              (int)renderTarget()._height - 1 - y);
          _pick_depth = pick_result.depth;
          _pick_id = pick_result.id;
          _picked.reset();
          LockScope ws;
          _picked = ws->document()->display();
          if (pick_result.id) {
            {
              ws->document()->display()->recurseDisplays(
                  [&](const std::shared_ptr<Display> &display) {
                    if (display->pick(pick_result.id)) {
                      LOG_DEBUG("pick Display");
                      _picked = display;
                    }
                  });
            }
            {
              if (ws->player && ws->document()->timeline()) {
                auto current_time = ws->player->time();
                for (auto &track : ws->document()->timeline()->tracks()) {
                  if (auto annotation_track =
                          std::dynamic_pointer_cast<AnnotationTrack>(track)) {
                    if (auto branch = annotation_track->branch(ws(), false)) {
                      for (auto &span : branch->spans()) {
                        if (span->start() <= current_time &&
                            span->start() + span->duration() >= current_time) {
                          for (auto &annotation : span->annotations()) {
                            if (auto scene_annotation =
                                    std::dynamic_pointer_cast<
                                        SceneAnnotationBase>(annotation)) {
                              if (scene_annotation->pick(pick_result.id)) {
                                LOG_DEBUG("pick SceneAnnotationBase");
                                _picked = scene_annotation;
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        });
        GlobalEvents::instance()->redraw();
      }
      break;
    }
    case QEvent::MouseButtonRelease: {
      _mouse_buttons = 0;
      auto *mouse = static_cast<QMouseEvent *>(event);
      if (mouse->button() == Qt::LeftButton && !_left_dragged) {
        auto modifiers = mouse->modifiers();
        pushAction([this, modifiers](const RenderWindowAsyncContext &context) {
          auto picked = _picked.lock();
          startOnMainThreadAsync([picked, modifiers]() {
            LockScope ws;
            auto s = ws->selection();
            LOG_DEBUG("modifiers " << modifiers);
            if ((modifiers & Qt::ShiftModifier) == Qt::ShiftModifier) {
              s.add(picked);
            } else if ((modifiers & Qt::ControlModifier) ==
                       Qt::ControlModifier) {
              s.toggle(picked);
            } else {
              s = picked;
            }
            if (s != ws->selection()) {
              // ActionScope ws("Pick");
              ws->selection() = s;
              ws->modified();
            }
          });
        });
        GlobalEvents::instance()->redraw();
      }
      break;
    }
    case QEvent::MouseMove: {
      auto *mouse = static_cast<QMouseEvent *>(event);
      if (event->type() == QEvent::MouseMove && mouse->buttons() != 0 &&
          mouse->buttons() == _mouse_buttons) {
        if (mouse->buttons() == Qt::LeftButton) {
          _left_dragged = true;
        }
        if (mouse->buttons() == Qt::RightButton) {
          LockScope ws;
          QPoint d = mouse->pos() - _mouse_position;
          double s =
              1.0f / std::tan((double)(fieldOfView() * M_PI / 180) * 0.5);
          double meters_per_pixel = (viewTarget() - viewPosition()).norm() /
                                    std::sqrt(1.0 * width() * height()) / s * 2;
          double dx = d.x() * meters_per_pixel;
          double dy = d.y() * meters_per_pixel;
          Eigen::Vector3d mx = (viewTarget() - viewPosition())
                                   .cross(Eigen::Vector3d::UnitZ())
                                   .normalized() *
                               -dx;
          Eigen::Vector3d my = (viewTarget() - viewPosition())
                                   .cross(Eigen::Vector3d::UnitZ())
                                   .cross(viewTarget() - viewPosition())
                                   .normalized() *
                               dy;
          viewTarget() += mx + my;
          viewPosition() += mx + my;
          if (event->type() == QEvent::MouseButtonRelease) {
            ws->modified();
          }
          GlobalEvents::instance()->redraw();
        }
        if (mouse->buttons() == Qt::MiddleButton) {
          LockScope ws;
          QPoint d = mouse->pos() - _mouse_position;
          double exponent = d.y() * 0.01;
          double factor = std::pow(0.5, exponent);
          viewPosition() =
              (viewPosition() - viewTarget()) * factor + viewTarget();
          if (event->type() == QEvent::MouseButtonRelease) {
            ws->modified();
          }
          GlobalEvents::instance()->redraw();
        }
      }
      _mouse_position = mouse->pos();
      break;
    }
  }
  if (((event->type() == QEvent::MouseButtonPress ||
        event->type() == QEvent::MouseButtonDblClick) &&
       (static_cast<QMouseEvent *>(event)->button() == Qt::LeftButton)) ||
      ((event->type() == QEvent::MouseMove) &&
       (static_cast<QMouseEvent *>(event)->buttons() & Qt::LeftButton)) ||
      ((event->type() == QEvent::MouseButtonRelease) &&
       (static_cast<QMouseEvent *>(event)->button() == Qt::LeftButton))) {
    auto *mouse = static_cast<QMouseEvent *>(event);
    int event_x = mouse->x();
    int event_y = mouse->y();
    int event_type = event->type();
    {
      pushAction(
          [event_type, event_x, event_y,
           this](const RenderWindowAsyncContext &context) {
            NoMessageScope m_scope;
            LockScope ws;
            if (event_type == QEvent::MouseButtonPress) {
              _interaction = Interaction();
              _interaction.id = _pick_id;
            }
            _interaction.current.view_matrix = _view_matrix;
            _interaction.current.projection_matrix = _projection_matrix;
            _interaction.current.x = event_x;
            _interaction.current.y = event_y;
            _interaction.current.center = viewPosition();
            Eigen::Vector4d p(
                _interaction.current.x * 2.0 / renderWidth() - 1.0,
                1.0 - _interaction.current.y * 2.0 / renderHeight(),
                _pick_depth * 2 - 1, 1.0);
            Eigen::Vector4d q =
                (_camera_block.projection_matrix * _camera_block.view_matrix)
                    .cast<double>()
                    .inverse() *
                p;
            _interaction.current.direction =
                (q.head(3) / q.w() - _interaction.current.center);
            _interaction.current.point =
                _interaction.current.center + _interaction.current.direction;

            if (event_type == QEvent::MouseButtonPress ||
                event_type == QEvent::MouseButtonDblClick) {
              _interaction.begin = _interaction.current;
              _interaction.previous = _interaction.current;
            }
            _interaction.pressed = (event_type == QEvent::MouseButtonPress ||
                                    event_type == QEvent::MouseButtonDblClick);
            _interaction.finished = (event_type == QEvent::MouseButtonRelease);
            bool interactive = false;
            if (auto picked = _picked.lock()) {
              if (auto p = std::dynamic_pointer_cast<Display>(picked)) {
                LOG_DEBUG("Display::interact");
                interactive = p->interact(_interaction);
              }
              if (auto p =
                      std::dynamic_pointer_cast<SceneAnnotationBase>(picked)) {
                LOG_DEBUG("SceneAnnotationBase::interact");
                ActionScope ws("Interact");
                interactive = p->interact(_interaction);
              }
              // if (_interaction.finished) {
              //   ActionScope ws("Interact");
              // }
            }
            if (!interactive) {
              interact(_interaction);
            }
            if (_interaction.finished) {
              startOnMainThreadAsync([]() { LockScope()->modified(); });
            }
            _interaction.previous = _interaction.current;
            GlobalEvents::instance()->redraw();
          },
          (event_type == QEvent::MouseMove) ? "move" : "");
      GlobalEvents::instance()->redraw();
    }
  }
}

void SceneWindow::interact(const Interaction &interaction) {
  LockScope ws;
  QPoint d(interaction.current.x - interaction.previous.x,
           interaction.current.y - interaction.previous.y);
  double degrees_per_pixel = 0.25;
  double f = degrees_per_pixel * M_PI / 180.0;
  double dx = d.x() * f;
  double dy = d.y() * f;
  viewPosition() -= viewTarget();
  viewPosition() =
      Eigen::AngleAxisd(-dx, Eigen::Vector3d::UnitZ()) * viewPosition();
  double pitch = std::max(
      -M_PI * 0.499,
      std::min(M_PI * 0.499, std::asin(viewPosition().normalized().z()) + dy));
  double distance = std::max(0.01, viewPosition().norm());
  viewPosition() = ((viewPosition().array() * Eigen::Vector3d(1, 1, 0).array())
                            .matrix()
                            .normalized() *
                        std::cos(pitch) +
                    Eigen::Vector3d(0, 0, std::sin(pitch))) *
                   distance;
  viewPosition() += viewTarget();
}

void SceneWindow::paintHUD(QPainter *painter) {
  if (LoaderThread::instance()->count() > 0) {
    static auto font = []() {
      QFont font;
      font.setPixelSize(32);
      return font;
    }();
    static auto text = []() {
      QStaticText text("Loading...");
      text.setPerformanceHint(QStaticText::AggressiveCaching);
      text.prepare(QTransform(), font);
      return text;
    }();
    int padding_x = 16;
    int padding_y = 8;
    painter->fillRect(_width - padding_x * 2 - text.size().width(), 0,
                      padding_x * 2 + text.size().width(),
                      padding_y * 2 + text.size().height(),
                      QBrush(QColor(0, 0, 0, 150)));
    painter->setPen(QPen(QBrush(Qt::white), 0));
    painter->drawStaticText(_width - padding_x - text.size().width(), padding_y,
                            text);
  }
}
