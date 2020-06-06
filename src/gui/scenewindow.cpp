// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "scenewindow.h"

#include "../core/workspace.h"

#include "../render/resource.h"
#include "../render/shader.h"
#include "../render/transformations.h"
#include "../render/uniformbuffer.h"

SceneWindow::SceneWindow()
    : _uniform_buffer(
          new UniformBuffer<CameraBlock>((size_t)UniformBindingPoint::camera)) {
}
void SceneWindow::renderWindowSync(const RenderWindowSyncContext &context) {
  LockScope ws;
  _bgcolor = ws->document()->display()->backgroundColor().toLinearVector4f();
  _camera_block.view_matrix =
      lookatMatrix(viewPosition(), viewTarget(), Eigen::Vector3d::UnitZ());
  float far = (100.0 + (viewPosition() - viewTarget()).norm() * 2.0);
  float near = far * 0.0001f;
  _camera_block.projection_matrix =
      projectionMatrix(1.0, _height * 1.0 / _width, near, far);
}
void SceneWindow::renderWindowAsync(const RenderWindowAsyncContext &context) {
  V_GL(glViewport(0, 0, _width, _height));
  V_GL(glClearColor(_bgcolor.x(), _bgcolor.y(), _bgcolor.z(), _bgcolor.w()));
  V_GL(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
  context.renderer->render(_camera_block, *context.render_list);
}
void SceneWindow::handleEvent(QEvent *event) {
  // LOG_DEBUG(event->type());
  switch (event->type()) {
  case QEvent::Wheel: {
    // ActionScope ws("Change View", shared_from_this(), true); // keep redo
    // stack
    LockScope ws;
    auto *wheel = static_cast<QWheelEvent *>(event);
    double degrees = wheel->angleDelta().y() * (1.0 / 8);
    double exponent = degrees / 90;
    double factor = std::pow(0.5, exponent);
    viewPosition() = (viewPosition() - viewTarget()) * factor + viewTarget();
    ws->modified();
    break;
  }
  /*case QEvent::Resize: {
    _mouse_buttons = 0;
    break;
}*/
  case QEvent::MouseButtonPress: {
    LOG_DEBUG("press");
    LockScope ws;
    auto *mouse = static_cast<QMouseEvent *>(event);
    _mouse_position = mouse->pos();
    _mouse_buttons = mouse->buttons();
    /*if (mouse->flags() & Qt::MouseEventCreatedDoubleClick) {
      _mouse_buttons = 0;
  }*/
    ws->redraw();
    break;
  }
  case QEvent::MouseButtonRelease: {
    _mouse_buttons = 0;
    break;
  }
  case QEvent::MouseMove: {
    auto *mouse = static_cast<QMouseEvent *>(event);
    if (event->type() == QEvent::MouseMove && mouse->buttons() != 0 &&
        mouse->buttons() == _mouse_buttons) {
      if (mouse->buttons() == Qt::LeftButton) {
        // ActionScope ws("Change View", shared_from_this(), true);// keep redo
        // stack
        LockScope ws;
        QPoint d = mouse->pos() - _mouse_position;
        double degrees_per_pixel = 0.25;
        double f = degrees_per_pixel * M_PI / 180.0;
        double dx = d.x() * f;
        double dy = d.y() * f;
        viewPosition() -= viewTarget();
        viewPosition() =
            Eigen::AngleAxisd(-dx, Eigen::Vector3d::UnitZ()) * viewPosition();
        double pitch =
            std::max(-M_PI * 0.499,
                     std::min(M_PI * 0.499,
                              std::asin(viewPosition().normalized().z()) + dy));
        double distance = std::max(0.01, viewPosition().norm());
        viewPosition() =
            ((viewPosition().array() * Eigen::Vector3d(1, 1, 0).array())
                     .matrix()
                     .normalized() *
                 std::cos(pitch) +
             Eigen::Vector3d(0, 0, std::sin(pitch))) *
            distance;
        viewPosition() += viewTarget();
        if (event->type() == QEvent::MouseButtonRelease) {
          ws->modified();
        }
        ws->redraw();
      }
      if (mouse->buttons() == Qt::RightButton) {
        // ActionScope ws("Change View", shared_from_this(), true);// keep redo
        // stack
        LockScope ws;
        QPoint d = mouse->pos() - _mouse_position;
        double meters_per_pixel = (viewTarget() - viewPosition()).norm() /
                                  std::sqrt(1.0 * width() * height());
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
        ws->redraw();
      }
      if (mouse->buttons() == Qt::MiddleButton) {
        // ActionScope ws("Change View", shared_from_this(), true);// keep redo
        // stack
        LockScope ws;
        QPoint d = mouse->pos() - _mouse_position;
        double exponent = d.y() * 0.01;
        double factor = std::pow(0.5, exponent);
        viewPosition() =
            (viewPosition() - viewTarget()) * factor + viewTarget();
        if (event->type() == QEvent::MouseButtonRelease) {
          ws->modified();
        }
        ws->redraw();
      }
    }
    _mouse_position = mouse->pos();
    break;
  }
  }
}
