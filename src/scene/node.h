// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "../core/document.h"
#include "../core/transformer.h"

#include <list>
#include <memory>

#include <Eigen/Dense>

class SceneNode;

struct SceneContext {
  std::vector<std::shared_ptr<SceneNode>> nodes;
};

class SceneNode : public std::enable_shared_from_this<SceneNode> {
private:
  std::mutex _mutex;
  bool _has_parent = false;
  bool _visible = true;
  Frame _frame;
  Eigen::Affine3d _pose = Eigen::Affine3d::Identity();
  std::list<std::weak_ptr<SceneNode>> _children;
  std::vector<std::shared_ptr<SceneNode>> _render_list;
  Eigen::Affine3d _frame_pose = Eigen::Affine3d::Identity();

public:
  SceneNode();
  SceneNode(const SceneNode &) = delete;
  SceneNode &operator=(const SceneNode &) = delete;
  ~SceneNode();
  void connect(const std::shared_ptr<SceneNode> &child);
  void renderSyncRecursive(const RenderSyncContext &context,
                           SceneContext &scene_context);
  virtual void renderSync(const RenderSyncContext &context);
  virtual void renderAsync(const RenderAsyncContext &context);
  Eigen::Affine3d renderPose() const { return _frame_pose * _pose; }
  const Eigen::Affine3d &framePose() const { return _frame_pose; }
  virtual bool pick(uint32_t id) const;
  virtual bool interact(const Interaction &interaction);
  const Frame &frame() const { return _frame; }
  void frame(const Frame &frame) { _frame = frame; }
  void frame(const std::string &name) { _frame.name(name); }
  void pose(const Eigen::Isometry3d &pose) { _pose = Eigen::Affine3d(pose); }
  void pose(const Eigen::Affine3d &pose) { _pose = pose; }
  const Eigen::Affine3d &pose() const { return _pose; }
  void show() { _visible = true; }
  void hide() { _visible = false; }
  bool visible() const { return _visible; }
  bool visible(bool v) { _visible = v; }
  template <class T, class... Args>
  std::shared_ptr<T> create(const Args &... args) {
    auto instance = std::make_shared<T>(args...);
    connect(instance);
    return instance;
  }
};
