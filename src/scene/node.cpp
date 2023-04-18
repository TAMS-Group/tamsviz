// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "node.h"

#include "../core/interaction.h"
#include "../core/workspace.h"
#include "material.h"

SceneNode::SceneNode() {}

SceneNode::~SceneNode() {}

void SceneNode::connect(const std::shared_ptr<SceneNode> &child) {
  if (child->_has_parent) {
    throw std::runtime_error("this scene node has already been added to a "
                             "different parent scene node");
  }
  std::lock_guard<std::mutex> lock(_mutex);
  _children.push_back(child);
  child->_has_parent = true;
}

bool SceneNode::pick(uint32_t id) const {
  for (auto &child : _render_list) {
    if (child->pick(id)) {
      return true;
    }
  }
  return false;
}

bool SceneNode::interact(const Interaction &interaction) {
  for (auto &child : _render_list) {
    if (child->interact(interaction)) {
      return true;
    }
  }
  return false;
}

void SceneNode::renderSyncRecursive(const RenderSyncContext &c,
                                    SceneContext &scene_context) {
  if (!_visible) {
    return;
  }
  auto context = c;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_frame.empty()) {
      if (auto p =
              _frame.pose(LockScope()->document()->display()->transformer)) {
        context.pose = *p;
      }
    }
    _frame_pose = context.pose;
    context.pose = context.pose * _pose;
    scene_context.nodes.push_back(shared_from_this());
  }
  renderSync(context);
  _render_list.clear();
  {
    std::lock_guard<std::mutex> lock(_mutex);
    for (auto it = _children.begin(); it != _children.end();) {
      if (auto child = it->lock()) {
        _render_list.push_back(child);
        it++;
      } else {
        it = _children.erase(it);
      }
    }
  }
  for (auto &child : _render_list) {
    child->renderSyncRecursive(context, scene_context);
  }
}

void SceneNode::renderSync(const RenderSyncContext &context) {}

void SceneNode::renderAsync(const RenderAsyncContext &context) {}
