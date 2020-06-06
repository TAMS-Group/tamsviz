// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "renderwindow.h"

#include "../render/renderer.h"

class SceneWindow : public RenderWindowBase {
  Eigen::Vector4f _bgcolor;
  std::shared_ptr<UniformBufferBase> _uniform_buffer;
  CameraBlock _camera_block;
  QPoint _mouse_position;
  int _mouse_buttons = 0;

public:
  SceneWindow();
  virtual void
  renderWindowSync(const RenderWindowSyncContext &context) override;
  virtual void
  renderWindowAsync(const RenderWindowAsyncContext &context) override;
  virtual void handleEvent(QEvent *event) override;
  PROPERTY(Eigen::Vector3d, viewPosition, Eigen::Vector3d(3, 3, 3));
  PROPERTY(Eigen::Vector3d, viewTarget, Eigen::Vector3d(0, 0, 0));
};
DECLARE_TYPE(SceneWindow, RenderWindowBase);
