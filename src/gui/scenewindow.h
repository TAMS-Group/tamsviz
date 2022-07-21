// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "renderwindow.h"

#include "../core/interaction.h"

#include "../render/renderer.h"

class MeshDisplayBase;
class Interaction;

class SceneWindow : public RenderWindowBase {
  Eigen::Vector4f _bgcolor;
  std::shared_ptr<UniformBufferBase> _uniform_buffer;
  CameraBlock _camera_block;
  Eigen::Matrix4d _view_matrix;
  Eigen::Matrix4d _projection_matrix;
  QPoint _mouse_position;
  bool _left_dragged = false;
  int _mouse_buttons = 0;
  int _multi_sampling = 0;
  std::mutex _action_mutex;
  std::vector<std::function<void(const RenderWindowAsyncContext &)>>
      _action_queue;
  std::string _action_tag;
  std::weak_ptr<Object> _picked;
  Interaction _interaction;
  double _pick_depth = 1.0;
  uint32_t _pick_id = 0;
  RenderTarget _render_target;
  void interact(const Interaction &interaction);
  void updateViewMatrix();
  void pushAction(
      const std::function<void(const RenderWindowAsyncContext &)> &action,
      const std::string &tag = "");

public:
  SceneWindow();
  virtual void
  renderWindowSync(const RenderWindowSyncContext &context) override;
  virtual void
  renderWindowAsync(const RenderWindowAsyncContext &context) override;
  virtual void handleEvent(QEvent *event) override;
  virtual void paintHUD(QPainter *painter) override;
  RenderTarget &renderTarget() { return _render_target; }
  virtual void composite(int target) override;
  PROPERTY(Eigen::Vector3d, viewPosition, Eigen::Vector3d(3, 3, 3));
  PROPERTY(Eigen::Vector3d, viewTarget, Eigen::Vector3d(0, 0, 0));
  PROPERTY(double, fieldOfView, 180 / M_PI);
};
DECLARE_TYPE(SceneWindow, RenderWindowBase);
